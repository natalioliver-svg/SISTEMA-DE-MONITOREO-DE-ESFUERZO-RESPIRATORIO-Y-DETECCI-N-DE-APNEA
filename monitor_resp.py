import csv                                  # guardar la sesión en un archivo CSV
import threading                            # leer la UART en paralelo al gráfico
import time                                 # marcas de tiempo y control de "sin datos"
from collections import deque               # listas de largo fijo

import serial                               # pyserial: acceso al puerto serie
import matplotlib.pyplot as plt             # ventana y gráfico
from matplotlib.animation import FuncAnimation  # actualiza el gráfico cada 50 ms
from matplotlib.ticker import MultipleLocator, FuncFormatter  # marcas del eje X
from matplotlib.patches import FancyBboxPatch    # cajas redondeadas del diseño

PUERTO = "COM13"                            # puerto del LPC (en Linux: /dev/ttyUSB0)
BAUD = 115200                               # tiene que coincidir con el firmware
VENTANA_S = 10                              # segundos que se ven a la vez
MUESTRA_S = 0.01                            # tiempo entre muestras (10 ms)
DAC_RETARDO_S = 1.0                         # CAMBIO: retardo entre la muestra y su salida por el DAC (estimado, ver nota)
MODO_PAGINA = False                         # False = el eje se desliza; True = salta de a 10 s (0-10, 10-20...)
MODO_CRUDO = False                          # True: imprime los bytes tal cual llegan (diagnóstico)
APNEA_S = 12                                # segundos sin respirar para declarar apnea (ref_apnea)
UMBRAL_ALTO = 3000                          # UMBRAL_ALTO del firmware: x por encima = inspiración
UMBRAL_BAJO = 2500                          # UMBRAL_BAJO del firmware: x debe bajar de acá

# paleta tipo monitor de ventilador (fondo oscuro, números celestes, botones grises)
C = {"fondo": "#0a0e13",                    # fondo de la ventana
     "barra": "#141a21",                    # barra superior
     "panel": "#10161c",                    # fondo del gráfico y de las tarjetas
     "borde": "#2b3540",                    # bordes y grilla
     "tenue": "#8aa0b3",                    # textos secundarios
     "cian": "#3ec1f0",                     # números y curva del sensor
     "dac": "#c77dff",                      # CAMBIO: curva de la salida del DAC
     "texto": "#e8edf2",                    # textos principales
     "boton": "#3a4048",                    # botones grises
     "boton_borde": "#5a626b",              # borde de los botones
     "rojo": "#e5383b",                     # alarma
     "naranja": "#f4a261",                  # umbral bajo y errores
     "verde": "#5fd38d"}                    # estado normal

tiempo = deque(maxlen=2000)                 # instante de cada muestra, en segundos desde el inicio
datos = deque(maxlen=2000)                  # señal x (12 bits); guarda 20 s, más que la ventana
dac_v = deque(maxlen=2000)                  # CAMBIO: valor del DAC (10 bits) multiplicado por 4 para compararlo con x
resp_t = deque(maxlen=5)                    # instantes de las últimas respiraciones
candado = threading.Lock()                  # evita leer y escribir datos a la vez
detener = threading.Event()                 # aviso al hilo lector para que termine
est = {"x": 0,                              # último valor del ADC recibido (12 bits)
       "cant": None,                        # último CANT_RESPI recibido (None = aún ninguno)
       "rpm": 0,                            # CANT_RESPI_UART: resp/min del último minuto completo
       "resp": 0,                           # respiraciones contadas desde que arrancó el script
       "validas": 0, "malas": 0,            # líneas entendidas y líneas descartadas
       "ultimo": 0.0,                       # hora de la última línea válida
       "inicio": 0.0,                       # hora de la primera línea D (0 = todavía no llegó)
       "t_m": 0.0,                          # instante (s) de la última muestra recibida
       "error": ""}                         # mensaje de error del puerto


def lector():                               # corre en un hilo aparte
    try:
        s = serial.Serial(PUERTO, BAUD, timeout=1)   # abre el puerto (8N1 por defecto)
    except serial.SerialException as e:     # puerto inexistente u ocupado
        est["error"] = f"No se pudo abrir {PUERTO}"  # lo muestra la ventana
        print(f"No se pudo abrir {PUERTO}: {e}")     # y también la consola
        return                              # sin puerto no hay nada que leer
    with s, open("sesion.csv", "w", newline="") as f:  # cierra los dos al terminar
        w = csv.writer(f)                   # escritor de CSV
        w.writerow(["t_s", "x_adc", "cant_respi", "resp_min", "v_dac"])  # CAMBIO: agrega la columna del DAC
        t0 = time.time()                    # instante cero de la sesión
        t_flush = t0                        # última vez que se guardó en disco
        s.reset_input_buffer()              # descarta lo que haya llegado antes de empezar
        try:
            while not detener.is_set():     # hasta que la ventana se cierre
                crudo = s.readline()        # bytes hasta '\n' (o 1 s de espera)
                if not crudo:               # pasó 1 s sin nada
                    continue                # volver a esperar
                if MODO_CRUDO:              # modo diagnóstico
                    print(crudo)            # ver los bytes: basura = baudrate o formato mal
                    continue                # no interpretar nada
                p = crudo.decode(errors="ignore").strip().split(",")  # "D,2048,3,18,512" -> lista
                try:
                    with candado:           # modifica datos y est sin que el gráfico los lea
                        if p[0] != "D" or len(p) < 4:   # solo líneas D con al menos 4 campos
                            est["malas"] += 1           # cualquier otra cosa se descarta
                            continue                    # y no se guarda en el CSV
                        x = int(p[1])               # valor del ADC, 12 bits (0 a 4095)
                        c = int(p[2])               # CANT_RESPI: respiraciones del minuto en curso
                        r = int(p[3])               # CANT_RESPI_UART: resp/min del último minuto
                        v = int(p[4]) if len(p) >= 5 else None   # CAMBIO: valor del DAC (0 a 1023); None si el firmware no lo manda
                        ahora = time.time()         # instante de llegada de la línea
                        # Las líneas llegan en ráfagas de 50 cada 0,5 s: dentro de la ráfaga las
                        # muestras se espacian 10 ms, y al empezar una nueva se re-sincroniza.
                        if est["inicio"] == 0.0:    # primera línea de la sesión
                            est["inicio"] = ahora   # el tiempo 0 del eje es este instante
                            t_m = 0.0
                        elif ahora - est["ultimo"] > 0.2:   # pasaron más de 0,2 s: empezó otra ráfaga
                            t_m = max(est["t_m"] + MUESTRA_S,
                                      (ahora - est["inicio"]) - 0.5)
                        else:                       # misma ráfaga: una muestra cada 10 ms
                            t_m = est["t_m"] + MUESTRA_S
                        est["t_m"] = t_m            # recuerda el instante de esta muestra
                        tiempo.append(t_m)          # guarda el tiempo
                        datos.append(x)             # agrega la muestra al gráfico
                        dac_v.append(v * 4 if v is not None else float("nan"))  # CAMBIO: el DAC (10 bits) x4 queda en la escala de 12 bits
                        est["x"] = x                # guarda el último valor
                        est["rpm"] = r              # guarda la frecuencia respiratoria
                        if est["cant"] is not None and c > est["cant"]:  # el contador subió
                            n = c - est["cant"]     # cuántas respiraciones llegaron en este bloque
                            est["resp"] += n        # suma al total de la sesión
                            for _ in range(n):      # una marca de tiempo por cada una
                                resp_t.append(ahora)
                        est["cant"] = c             # si bajó (reinicio por minuto) solo se actualiza
                        est["validas"] += 1         # una línea más entendida
                        est["ultimo"] = ahora       # sirve para detectar "sin datos" y nuevas ráfagas
                    w.writerow([round(time.time() - t0, 2), x, c, r, v])  # CAMBIO: tiempo y los cuatro valores
                except (ValueError, IndexError):    # línea cortada o con basura
                    est["malas"] += 1               # se cuenta y se ignora
                if time.time() - t_flush > 1:       # cada segundo
                    f.flush()                       # guarda en disco lo acumulado
                    t_flush = time.time()
        except serial.SerialException as e:         # se desconectó el cable
            est["error"] = "Se perdió la conexión con el puerto"
            print(e)


hilo = threading.Thread(target=lector, daemon=True)  # hilo lector
hilo.start()                                # arranca la lectura

fig = plt.figure(figsize=(13, 7.5), facecolor=C["fondo"])   # ventana oscura y ancha
fig.canvas.manager.set_window_title("Monitor de esfuerzo respiratorio")


def caja(x, y, w, h, relleno, borde, grosor=1.0):   # caja redondeada en coordenadas de la ventana (0 a 1)
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.008",
                       transform=fig.transFigure, facecolor=relleno,
                       edgecolor=borde, linewidth=grosor)
    fig.add_artist(p)                       # la agrega a la ventana
    return p                                # se devuelve para poder cambiarle el color


# barra superior: título, estado y caja de alerta
caja(0.0, 0.93, 1.0, 0.07, C["barra"], C["barra"])                       # fondo de la barra
fig.text(0.02, 0.965, "Monitor de esfuerzo respiratorio", color=C["texto"],
         fontsize=15, weight="bold", va="center")                        # título
estado = fig.text(0.89, 0.965, "", fontsize=12, weight="bold",
                  ha="right", va="center", color=C["tenue"])             # mensaje de estado
alerta = caja(0.905, 0.9425, 0.065, 0.045, "#2a323b", C["borde"])        # cuadro de alerta
alerta_txt = fig.text(0.9375, 0.965, "i", fontsize=16, weight="bold",
                      ha="center", va="center", color=C["tenue"])        # "i" normal, "!" en apnea

# gráfico de la señal
ax = fig.add_axes([0.05, 0.27, 0.60, 0.62], facecolor=C["panel"])        # posición del gráfico
(linea,) = ax.plot([], [], lw=1.8, color=C["cian"])                      # curva del sensor (celeste)
(linea_dac,) = ax.plot([], [], lw=1.4, color=C["dac"], alpha=0.9)        # CAMBIO: curva de la salida del DAC (violeta), superpuesta
ax.axhline(UMBRAL_ALTO, color=C["rojo"], ls="--", lw=0.8, alpha=0.7)     # umbral de inspiración
ax.axhline(UMBRAL_BAJO, color=C["naranja"], ls="--", lw=0.8, alpha=0.7)  # umbral para habilitar la siguiente
ax.set_xlim(0, VENTANA_S)                   # arranca mostrando de 0 a 10 s
ax.set_ylim(0, 4095)                        # rango completo del ADC de 12 bits
ax.xaxis.set_major_locator(MultipleLocator(1))        # marca con número cada 1 s
ax.xaxis.set_minor_locator(MultipleLocator(0.1))      # marca chica cada 100 ms
ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:.0f}"))  # números enteros
ax.set_yticks([])                           # sin números en el eje Y
ax.set_xlabel("Tiempo (s)", color=C["tenue"])
ax.set_ylabel("Esfuerzo Respiratorio Torácico", color=C["tenue"])
ax.tick_params(colors=C["tenue"], which="both")       # marcas en gris azulado
ax.grid(True, color=C["borde"], lw=0.6)               # grilla tenue
for lado in ax.spines.values():             # marco del gráfico
    lado.set_color(C["borde"])
leyenda = ax.legend([linea, linea_dac], ["Sensor (ADC)", "Salida del DAC"],   # CAMBIO: leyenda de las dos curvas
                    loc="upper left", facecolor=C["panel"], edgecolor=C["borde"],
                    labelcolor=C["tenue"], fontsize=9, framealpha=0.9)


def tarjeta(y, titulo, unidad):             # tarjeta con número grande
    caja(0.68, y, 0.30, 0.19, C["panel"], C["borde"])
    fig.text(0.692, y + 0.155, titulo, color=C["cian"], fontsize=13, weight="bold", va="center")
    fig.text(0.692, y + 0.125, unidad, color=C["tenue"], fontsize=10, va="center")
    return fig.text(0.965, y + 0.025, "--", color=C["cian"], fontsize=44,
                    weight="bold", ha="right", va="bottom")   # el número se actualiza después


num_rpm = tarjeta(0.70, "FR", "resp/min")                   # frecuencia respiratoria
num_resp = tarjeta(0.50, "Respiraciones", "detectadas")     # contador
num_seg = tarjeta(0.30, "Sin respirar", "s")                # segundos desde la última respiración


def boton(i, etiqueta, valor):              # botón gris con el valor en grande
    x = 0.03 + i * 0.1925
    caja(x, 0.05, 0.17, 0.15, C["boton"], C["boton_borde"], 1.2)
    fig.text(x + 0.085, 0.172, etiqueta, color="#c9d3dc", fontsize=10, ha="center", va="center")
    fig.text(x + 0.085, 0.105, valor, color=C["texto"], fontsize=22, weight="bold",
             ha="center", va="center")


boton(0, "Umbral alto", str(UMBRAL_ALTO))   # los 5 botones muestran la configuración
boton(1, "Umbral bajo", str(UMBRAL_BAJO))
boton(2, "Apnea (s)", str(APNEA_S))
boton(3, "Ventana (s)", str(VENTANA_S))
boton(4, "Muestreo (Hz)", str(int(round(1 / MUESTRA_S))))
info = fig.text(0.5, 0.018, "", ha="center", fontsize=9, color=C["tenue"])  # contadores abajo


def actualizar(_):                          # se llama cada 50 ms
    with candado:                           # copia consistente de los datos
        t = list(tiempo)                    # instantes de las muestras
        y = list(datos)                     # señal de la ventana
        d = list(dac_v)                     # CAMBIO: valores del DAC
        e = dict(est)                       # estado actual
        tr = list(resp_t)                   # instantes de las últimas respiraciones
    if t:                                   # solo si ya llegó algo
        linea.set_data(t, y)                # redibuja la curva del sensor
        linea_dac.set_data([ti + DAC_RETARDO_S for ti in t], d)   # CAMBIO: el DAC se dibuja corrido por su retardo
        if MODO_PAGINA:                     # modo página: 0-10, después 10-20, 20-30...
            lo = (int(t[-1]) // VENTANA_S) * VENTANA_S
        else:                               # modo deslizante: el eje avanza con la señal
            lo = max(0.0, t[-1] - VENTANA_S)
        ax.set_xlim(lo, lo + VENTANA_S)     # mueve la ventana del eje X
    ahora = time.time()                     # instante actual

    # resp/min: lo calcula el firmware (CANT_RESPI_UART); es 0 hasta cumplir el primer minuto
    rpm_valido = e["rpm"] > 0 or (e["inicio"] > 0 and ahora - e["inicio"] > 60)
    num_rpm.set_text(str(e["rpm"]) if rpm_valido else "--")
    num_resp.set_text(str(e["resp"]))

    # apnea: segundos desde la última respiración (o desde que llegó el primer dato)
    t_ref = tr[-1] if tr else e["inicio"]
    seg = (ahora - t_ref) if e["inicio"] else 0
    apnea = bool(e["inicio"]) and seg > APNEA_S
    num_seg.set_text(str(int(seg)))
    num_seg.set_color(C["rojo"] if apnea else C["cian"])         # pasa a rojo en apnea

    sin_datos = (ahora - e["ultimo"]) > 2   # más de 2 s sin líneas válidas
    if e["error"]:                          # el puerto falló
        fig.patch.set_facecolor("#2a1a08")
        estado.set_text(e["error"]); estado.set_color(C["naranja"])
        alerta.set_facecolor("#2a323b"); alerta_txt.set_text("i"); alerta_txt.set_color(C["tenue"])
    elif sin_datos:                         # el puerto abre pero no llega nada válido
        fig.patch.set_facecolor(C["fondo"])
        estado.set_text("Esperando datos de la UART..."); estado.set_color(C["tenue"])
        alerta.set_facecolor("#2a323b"); alerta_txt.set_text("i"); alerta_txt.set_color(C["tenue"])
    elif apnea:                             # alarma activa
        fig.patch.set_facecolor("#2a0b0d")
        estado.set_text(f"APNEA: {int(seg)} s sin respirar"); estado.set_color(C["rojo"])
        alerta.set_facecolor(C["rojo"]); alerta_txt.set_text("!"); alerta_txt.set_color("white")
    else:                                   # funcionamiento normal
        fig.patch.set_facecolor(C["fondo"])
        estado.set_text("Monitoreo normal"); estado.set_color(C["verde"])
        alerta.set_facecolor("#2a323b"); alerta_txt.set_text("i"); alerta_txt.set_color(C["tenue"])
    info.set_text(f"ADC {e['x']}    líneas válidas: {e['validas']}    descartadas: {e['malas']}"
                  f"    DAC con retardo de {DAC_RETARDO_S:g} s")   # CAMBIO: recuerda el retardo usado
    return linea, linea_dac, num_rpm, num_resp, num_seg, estado, info   # CAMBIO: incluye la curva del DAC


ani = FuncAnimation(fig, actualizar, interval=50, cache_frame_data=False)  # animación
plt.show()                                  # bloquea hasta cerrar la ventana
detener.set()                               # avisa al lector que termine
hilo.join(timeout=3)                        # espera a que cierre el CSV