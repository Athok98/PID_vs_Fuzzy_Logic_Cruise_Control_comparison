from simpful import *
import matplotlib.pyplot as plt
import pid_Reg_Temp
import fuzzy_Reg_Temp

from bokeh.layouts import gridplot
from bokeh.plotting import figure
from bokeh.io import output_file, show
from bokeh.layouts import column
from bokeh.models import Div


# Symulacja
T1 = 30          # Czas symulacji, s
Tp1 = 0.1            # Czas próbkowania, s
N = int(T1/Tp1) + 1   # Ilość próbek

czas_Tablica = [0.0]  # tablica czasu

# Sterowanie Rozmyte
tablica_Vel_Fz = [0.0]  # tablica predkosci
tablica_e_Fz = [0.0]  # tablica uchybu
tablica_ce_Fz = [0.0]   #tablica różnicy uchybu
tablica_u_Fz = [0.0]  # tablica sygnału sterującego
ce_Fz = 0.0  # zmiana uchybu

# Sterowanie PID
tablica_Vel_PID = [0.0]  # tablica prędkości
tablica_e_PID = [0.0]  # tablica uchybu
tablica_ce_PID = [0.0]  # tablica zmiany uchybu
tablica_u_PID = [0.0]  # tablica sygnału sterującego
ce_PID = 0.0  # zmiana uchybu

# Stałe
velocity = 50    # prędkość do osiągnięcia, m/s
velocity0 = 0   # prędkość początkowa, m/s
CoDrag = 0.3301029 #"współczynnik" - siła oporu powietrza dla naszego pojazdu przed uwzględnieniem zmiennej prędkości
Froll = 201.1 #siła oporu toczenia przeciwdziałająca pojazdowi
mass = 2050 #masa pojazdu

#Nastawy Regulatora
kp_1 = 3 # wzmocnienie
Ti_1 = 0.3 # czas całkowania
Td_1 = 0.1 #czas różniczkowania

F = [0.0]    # zmienne do mocy silnika
FEngineMin = 0.0      # minimalna moc silnika / ewentualnie hamowania silnikiem na minusie
FEngineMax = 1065     # maksymalna moc silnika


MaxVal = FEngineMax


F_Fz = 0.1  # procentowe sterowanie
Fc_Fz = 0.0  # zmiana sterowania

F_PID = 0.1  # procentowe sterowanie
Fc_PID = 0.0  # zmiana sterowania


Regulator = pid_Reg_Temp.PID(kp_1, Ti_1, Td_1, velocity, Tp1)


#  obliczanie maksymalnej prędkości
Vmax = pow((FEngineMax - Froll)/0.5 * 1.293 * 0.23 * 2.22, 1/2)


for n in range(1, N):
    czas_Tablica.append(n * Tp1)
    output, error = Regulator.compute(tablica_Vel_PID[-1])
    Fdrag = CoDrag * pow(tablica_Vel_PID[-1], 2)
    tablica_u_PID.append(max(min(output, FEngineMax), FEngineMin))
    tablica_Vel_PID.append(max(min(tablica_Vel_PID[-1] + ((tablica_u_PID[-1] - (Froll + Fdrag)) * (10 / mass)), Vmax),
                               velocity0))


    #  sterowanie
    #tablica_u_Fz.append(F_Fz * MaxVal)  # ustawianie siły silnika
    tablica_u_Fz.append(max(min(F_Fz * MaxVal, FEngineMax), FEngineMin))


    tablica_Vel_Fz.append(max(min(tablica_Vel_Fz[-1] + ((tablica_u_Fz[-1] - (Froll + Fdrag)) * (10 / mass)), Vmax),
                              velocity0))  # obliczanie prędkości


    # uchyb z maksymalnymi granicami <-1,1>

    error_last_Fz = tablica_e_Fz[-1]
    tablica_e_Fz.append((tablica_Vel_Fz[-1] - velocity) / Vmax)

    error_last_PID = tablica_e_PID[-1]
    tablica_e_PID.append((tablica_Vel_PID[-1] - velocity) / Vmax)

    # zmiana uchybu z maksymalnymi granicami
    if len(tablica_e_Fz) > 2:
        ce_Fz = tablica_e_Fz[-1] - error_last_Fz

        ce_PID = tablica_e_PID[-1] - error_last_PID

    tablica_ce_Fz.append(ce_Fz)
    tablica_ce_PID.append(ce_PID)

    fuzzy_Reg_Temp.Hfs.set_variable("uchyb", tablica_e_Fz[-1])
    fuzzy_Reg_Temp.Hfs.set_variable("zm_uchyb", ce_Fz)
    Fc_Fz = fuzzy_Reg_Temp.Hfs.Mamdani_inference(['zadany']).get('zadany')
    fuzzy_Reg_Temp.Hfs.set_variable("zadany", Fc_Fz)

    F_Fz = F_Fz + Fc_Fz
    if F_Fz > 1:
        F_Fz = 1
    if F_Fz < 0:
        F_Fz = 0

"""
figure, axis = plt.subplots(2, 2)

axis[0, 0].plot(czas_Tablica, tablica_Vel_Fz, label='Fuzzy')
axis[0, 0].plot(czas_Tablica, tablica_Vel_PID, label='PID')
axis[0, 0].set_title("Predkość [m/s]")

axis[0, 1].plot(czas_Tablica, tablica_ce_Fz, label='Fuzzy')
axis[0, 1].plot(czas_Tablica, tablica_ce_PID, label='PID')
axis[0, 1].set_title("Zmiana Uchybu")

axis[1, 0].plot(czas_Tablica, tablica_e_Fz, label='Fuzzy')
axis[1, 0].plot(czas_Tablica, tablica_e_PID, label='PID')
axis[1, 0].set_title("Uchyb")

axis[1, 1].plot(czas_Tablica, tablica_u_Fz, label='Fuzzy')
axis[1, 1].plot(czas_Tablica, tablica_u_PID, label='PID')
axis[1, 1].set_title("Siła z silnika")

axis[0, 0].legend()
axis[0, 1].legend()
axis[1, 0].legend()
axis[1, 1].legend()
plt.show()
"""

# create a new plot
s1 = figure(title="Prędkość(t)", background_fill_color="#fafafa", x_axis_label='Czas [s]', y_axis_label='Prędkość [m/s]')
s1.line(czas_Tablica, tablica_Vel_Fz, color="#53777a", legend_label="Fuzzy")
s1.line(czas_Tablica, tablica_Vel_PID, color="#c02942", legend_label="PID")


s2 = figure(title="Zmiana uchybu(t)", background_fill_color="#fafafa", x_axis_label='Czas [s]', y_axis_label='Zmiana uchybu')
s2.line(czas_Tablica, tablica_ce_Fz, color="#53777a", legend_label="Fuzzy")
s2.line(czas_Tablica, tablica_ce_PID, color="#c02942", legend_label="PID")

s3 = figure(title="Uchyb(t)", background_fill_color="#fafafa", x_axis_label='Czas [s]', y_axis_label='Uchyb')
s3.line(czas_Tablica, tablica_e_Fz, color="#53777a", legend_label="Fuzzy")
s3.line(czas_Tablica, tablica_e_PID, color="#c02942", legend_label="PID")

s4 = figure(title="Siła z silnika(t)", background_fill_color="#fafafa", x_axis_label='Czas [s]', y_axis_label='Moc silnika [N]')
s4.line(czas_Tablica, tablica_u_Fz, color="#53777a", legend_label="Fuzzy")
s4.line(czas_Tablica, tablica_u_PID, color="#c02942", legend_label="PID")

grid = gridplot([[s1, s2], [s3, s4]], width=500, height=500)

#show(grid)

div = Div(text="""Your <a href="https://en.wikipedia.org/wiki/HTML">HTML</a>-supported text is initialized with the <b>text</b> argument.  The
remaining div arguments are <b>width</b> and <b>height</b>. For this example, those values
are <i>200</i> and <i>100</i>, respectively.""",
width=200, height=100)

title = Div(text=""" <font size="+2"> Porównanie regulatorów typu PID i Fuzzy dla tempomatu modelu samochodu Tesla Model 3. </font>""", width=1000, height=50)

show(column(title, grid))


#show(column(title,grid))
#Porównanie regulatorów typu PID i Fuzzy dla tempomatu samochodu Tesla Model 3