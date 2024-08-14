import Pkg
Pkg.add("FileIO")
Pkg.add("ImageMagick")
using Agents, Agents.Pathfinding
using FileIO  # Para cargar y guardar imágenes
using ImageMagick  # Para la conversión de imágenes

# 1. Convertir la imagen PNG a BMP
# Cargar la imagen PNG
img = load("LABERINTO.png")

# Guardar la imagen en formato BMP
save("LABERINTO.bmp", img)

# 2. Usar la imagen BMP en la simulación

# Cargar la imagen en formato BMP
maze_map = load("LABERINTO.bmp");

# Se define un agente que se moverá en un espacio de grilla bidimensional
@agent struct Walker(GridAgent{2}) end 

# Definir un umbral para considerar un píxel como suficientemente blanco
threshold = 0.5  # Puedes ajustar este valor según el nivel de ruido en la imagen

function initialize_model(maze_map)
    # Convertir la imagen en un mapa de celdas transitables (true) y no transitables (false)
    # Ahora se considera transitable (true) si el pixel es blanco (todos los componentes RGB son máximos)
    maze = BitArray(map(x -> x.r > threshold && x.g > threshold && x.b > threshold, maze_map))
    
    # Crear el espacio de grilla bidimensional
    space = GridSpace(size(maze); periodic = false)
    
    # Crear el objeto A* para encontrar el camino más corto
    pathfinder = AStar(space; walkmap=maze, diagonal_movement=false)
    
    # Crear el modelo basado en agentes
    model = StandardABM(Walker, space; agent_step!)
    
    # Agregar un agente en la posición inicial, ajusta esta posición según sea necesario
    add_agent!((1, 1), model)
    
    # Planear la ruta hacia una posición objetivo, ajusta la posición de destino según sea necesario
    plan_route!(model[1], (10, 10), pathfinder)

    return model, pathfinder
end

# Definir el comportamiento del agente en cada paso de la simulación
agent_step!(agent, model) = move_along_route!(agent, model, pathfinder)

# Inicializar el modelo
model, pathfinder = initialize_model(maze_map)

# Visualizar la simulación
using CairoMakie

abmvideo(
    "maze7.mp4",  # Nombre del archivo de video
    model; 
    figurekwargs = (size =(180,196),),  # Tamaño de la figura en la que se visualizará la simulación
    frames=600,  # Número de cuadros que se renderizarán en el video
    framerate=10,  # Cuadros por segundo
    agent_color=:black,  # Cambia el color del agente a negro
    agent_size=11,  # Tamaño del agente
    heatarray = _ -> pathfinder.walkmap, 
    add_colorbar = false,  # Omitir la barra de colores
)
