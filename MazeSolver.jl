import Pkg; Pkg.add("FileIO")
import Pkg; Pkg.add("ImageMagick")
using Agents, Agents.Pathfinding
using FileIO # To load images you also need ImageMagick available to your project
using ImageMagick

# 1. Convertir la imagen PNG a BMP
# Cargar la imagen PNG
img = load("LABERINTO.png")

# Guardar la imagen en formato BMP
save("LABERINTO.bmp", img)

# 2. Usar la imagen BMP en la simulación

# Cargar la imagen en formato BMP
maze_map = load("LABERINTO.bmp");

# Definir un umbral para considerar un píxel como suficientemente blanco
threshold = 0.5  # Puedes ajustar este valor según el nivel de ruido en la imagen

# Se define un agente que se movera en un espacio de grilla bidimensional
@agent struct Walker(GridAgent{2}) end 

function initialize_model(maze_map)
    # map hace que se aplique a cada pixel de la imagen, x es un pixel luego devuelve true si el pixel es blanco y false si no lo es
    # map es un arreglo de arreglos de booleanos
    # BitArray convierte map a un arreglo especia que almacena los valores booleanos de manera eficiente
    maze = BitArray(map(x -> x.r > threshold && x.g > threshold && x.b > threshold, maze_map))
    # size(maze) devuelve el numero de filas y columnas de la matriz maze
    # periodic = false indica que el espacio no es periodico lo cual indica que el agent no puede salir de los limites del espacio
    # GridSpace crea un espacio de grilla bidimensional
    space = GridSpace(size(maze); periodic = false)
    # AStar es para encontrar el camino mas corto entre dos puntos
    # space es el espacio en el que se movera el agente
    # walkmap obtiene el mapa y define que celdas del espacio son accesibles y cuales no
    # diagonal_movement=false indica que el agente no puede moverse en diagonal
    pathfinder = AStar(space; walkmap=maze, diagonal_movement=false)
    # StandardABM inicia un modelo de simulacion basado en agentes
    # Walker es el tipo de agente que se movera en el espacio
    # space es el espacio en el que se movera el agente
    # agent_step! es la funcion que se ejecutara en cada paso de la simulacion
    # El signo (!) indica que la funcion agent_step! modifica el estado del agente
    model = StandardABM(Walker, space; agent_step!)
    # add_agent! agrega un agente en la posicion (1, 4) al modelo 
    add_agent!((5, 110), model)
    # plan_route! es una funcion que calcula la ruta mas corta entre dos puntos 
    # Como los agentes en el modelo se almacenan en una lista, se accede al agente en la posicion 1 que añadido anteriormente
    # (41, 32) es la posicion la posicion a la que se quiere llegar
    # pathfinder es el objeto que se encarga de encontrar la ruta mas corta
    plan_route!(model[1], (199, 70), pathfinder)

    return model, pathfinder
end

# agent_step! es la funcion que se ejecutara en cada paso de la simulacion, de la cual recibe como argumentos el agente y el modelo
# move_along_route! se ejecuta dentro de agent_step! y mueve al agente a lo largo de la ruta calculada por pathfinder, actualiza la posición 
# del agente basándose en la ruta que fue planificada con plan_route!
agent_step!(agent, model) = move_along_route!(agent, model, pathfinder)




# Se inicializa el modelo y esto devuelve el modelo y el pathfinder
model, pathfinder = initialize_model(maze_map)

using CairoMakie

abmvideo(
    "hola.mp4", # Nombre del archivo de video
    model; # contiene la información sobre los agentes, su entorno (el espacio de grilla), y las rutas que seguirán.
    figurekwargs = (size =(180,196),), # tamaño de la figura en la que se visualizará la simulación
    frames=600, # número de cuadros que se renderizarán en el video
    framerate=10, # cuadros por segundo
    agent_color=:red, # color del agente
    agent_size=11, # tamaño del agente
    # pathfinder.walkmap contiene un mapa binario (de valores true y false) que indica las áreas transitables y no transitables dentro del laberinto
    # heatarray es un arreglo de valores que se utilizará para colorear el mapa de calor
    # _ -> es una función anónima que se utiliza para acceder a la variable pathfinder.walkmap
    #  Este "heatmap" o heatarray muestra gráficamente las áreas del laberinto que son transitables (true) y aquellas que son obstáculos (false). Al usar una función anónima, se asegura que el mismo 
    # walkmap se aplique en cada frame de la simulación, permitiendo visualizar el laberinto y las rutas que los agentes pueden seguir.
    heatarray = _ -> pathfinder.walkmap, 
    add_colorbar = true,
)
