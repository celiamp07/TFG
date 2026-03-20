#include "trajectory_generator.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp; 

void TrajectoryGenerator::parseInputParameters(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);
    
    // Ahora guardas directamente en los atributos de la clase
    centroid = config["input_parameters"]["centroid"].as<std::vector<double>>();
    radius = config["input_parameters"]["radius"].as<double>();
    height = config["input_parameters"]["height"].as<double>();
    inclination = config["input_parameters"]["inclination"].as<double>() * M_PI / 180.0;
    resolution = config["input_parameters"]["resolution"].as<int>();
    speed = config["input_parameters"]["speed"].as<double>();
    start_position = config["input_parameters"]["start_position"].as<double>() * M_PI / 180.0;
    end_position = config["input_parameters"]["end_position"].as<double>() * M_PI / 180.0;
    initial_joint_positions = config["initial_joint_positions"].as<std::vector<double>>();
}

//Constructor Vacío
TrajectoryGenerator::TrajectoryGenerator(){ 
    centroid = {0.0, 0.0, 0.0};
    radius = 0;
    height = 0;
    inclination = 0;
    resolution = 0;
    start_position = 0;
    end_position = 0;
    speed = 0;
}

// Inicializa los miembros de la clase, la inclinación se convierte de grados a radianes
TrajectoryGenerator::TrajectoryGenerator(const std::vector<double>& centroid_, double radius_, double height_, double inclination_, int resolution_, double start_position_, double end_position_, double speed_) {
    centroid = centroid_;
    radius = radius_;
    height = height_;
    inclination = inclination_ * M_PI / 180.0;
    resolution = resolution_;
    start_position = start_position_ * M_PI / 180.0;
    end_position = end_position_ * M_PI / 180.0;
    speed = speed_;
}

std::vector<Point> TrajectoryGenerator::generateCircularTrajectory() {
    std::vector<Point> trajectory; // Inicializa un vector para almacenar los puntos generados 
    double angle_increment = (end_position - start_position) / resolution; // Incremento del ángulo, para generar puntos equidistantes a lo largo de la circunferencia. Calcula el ángulo entre cada punto consecutivo de la circunferencia
    
    //// Calcula el ángulo inicial en función de la posición de partida
    //double start_angle = atan2(start_position_[1] - centroid_[1], start_position_[0] - centroid_[0]);
   
    // Generar los puntos de la trayectoria entre start_position_ y end_position_
        for (int i = 0; i < resolution; ++i) { // Bucle que genera los puntos de la circunferencia, itera de 0 a resolution-1
            // Calcula t (ángulo actual del punto que está calculando) y las coordenadas x, y, z de cada punto de la circunferencia
            double t = start_position + i * angle_increment;
            double x = radius * cos(t);
            double y = radius * sin(t);
            double z = height; // Altura constante

            // Ajustar al centro y altura
            Point p;
            p.x = centroid[0] + x;
            p.y = centroid[1] + y;
            p.z = centroid[2] + z;

            trajectory.push_back(p); // Añade el punto generado en el vector de trayectoria 
        }

    return trajectory;
}

double TrajectoryGenerator::calculatePointDuration() const {
    double circumference = (end_position - start_position) * radius; // Calcula la circunferencia completa
    double total_duration = circumference / speed; // Calcula el tiempo total para completar una vuelta
    return total_duration / resolution; // Divide el tiempo total por el número de puntos para obtener la duración por punto
}

// Función para generar y visualizar la trayectoria

std::vector<Point> TrajectoryGenerator::handleGenerate(bool debug) {
    std::cout << "Generando trayectoria con los siguientes parámetros (desde memoria):" << std::endl;
    
    // Imprimir los parámetros directamente usando los nombres de las variables de la clase
    std::cout << "Centro de la circunferencia: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")\n";
    std::cout << "Radio de la circunferencia: " << radius << "\n";
    std::cout << "Altura: " << height << "\n";
    std::cout << "Inclinación vertical (radianes en memoria): " << inclination << "\n";
    std::cout << "Resolución (número de puntos): " << resolution << "\n";
    std::cout << "Posición inicial (radianes en memoria): " << start_position << "\n";
    std::cout << "Posición final (radianes en memoria): " << end_position << "\n";
    std::cout << "Velocidad deseada (mm/s): " << speed << "\n";

    // Llamamos a la función de la clase y guardamos en el atributo privado
    generated_points = generateCircularTrajectory();

    // Verificar si se generaron los puntos
    if (generated_points.empty()) {
        std::cerr << "Error: No se generaron puntos de la trayectoria.\n";
        return generated_points;
    }

    // Calcular la duración por punto
    double point_duration = calculatePointDuration();

    if (debug) {
        std::cout << "Duración por punto: " << point_duration << " segundos" << std::endl;
    }

    // Imprimir los puntos generados para verificación
    if (debug) {
        std::cout << "Presione Enter para ver los puntos generados..." << std::endl;
        std::cin.get();
        std::cout << "Puntos generados:" << std::endl;
        for (size_t i = 0; i < generated_points.size(); ++i) {
            std::cout << "Punto " << i << ": (" 
                      << generated_points[i].x << ", " 
                      << generated_points[i].y << ", " 
                      << generated_points[i].z << ")" << std::endl;
        }
    }

    // Guardar en el archivo de texto (para compatibilidad con el resto del código actual)
    std::ofstream outFile("trajectory_points.txt");
    if (!outFile.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo para escribir los puntos.\n";
        return generated_points;
    }

    for (const auto& point : generated_points) {
        outFile << point.x << " " << point.y << " " << point.z << std::endl;
    }
    outFile.close();

    return generated_points;
}

// Función para leer los parámetros de entrada desde el teclado
void TrajectoryGenerator::readFromConsole() {
    // Inicializar los vectores con 3 elementos
    centroid.resize(3);  // Asegurarse de que el vector tenga espacio para 3 elementos

    std::cout << "Ingrese el centro de la circunferencia (cx, cy, cz): ";
    std::cin >> centroid[0] >> centroid[1] >> centroid[2];
    std::cout << "Ingrese el radio de la circunferencia: ";
    std::cin >> radius;
    std::cout << "Ingrese la altura: ";
    std::cin >> height;
    std::cout << "Ingrese la inclinación vertical en grados: ";
    std::cin >> inclination;
    std::cout << "Ingrese la resolución (número de puntos): ";
    std::cin >> resolution;
    std::cout << "Ingrese la posición de inicio en grados (0-360): ";
    std::cin >> start_position;
    std::cout << "Ingrese la posición de fin en grados (0-360): ";
    std::cin >> end_position;
    std::cout << "Ingrese la velocidad deseada (mm/s): ";
    std::cin >> speed;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

// Función para cargar los puntos desde el archivo txt -> Para hacer el plot
bool TrajectoryGenerator::loadDataPlot(const std::string& filename, std::vector<Point>& points) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream stream(line);
        Point p;
        if (stream >> p.x >> p.y >> p.z) {
            points.push_back(p);
        }
        else {
            std::cerr << "Línea con formato incorrecto: " << line << std::endl;
        }
    }

    file.close();
    return true;
}

// Función para trazar los puntos
bool TrajectoryGenerator::handlePlot(const std::string& filename) {
    std::cout << "Presione Enter para continuar y ver el gráfico de los puntos generados..." << std::endl;
    std::cin.get();
    std::cout << "Generando gráfico..." << std::endl;
    
    std::vector<Point> trajectory_points;
    if (!loadDataPlot("trajectory_points.txt", trajectory_points)) {
        std::cerr << "Error: No se encontraron puntos en el archivo." << std::endl;
        return false;
    }

    std::vector<double> x, y, z;

    for (const auto& point : trajectory_points) {
        x.push_back(point.x);
        y.push_back(point.y);
        z.push_back(point.z);
    }

    plt::figure();
    // Dibujar todos los puntos 
    plt::named_plot("Puntos de la trayectoria", x, y, "b.");
    plt::scatter(x, y, 0.1); // Ajusta el tamaño de los marcadores

    // Incluir la altura en el título
    std::ostringstream title;
    title << "Trayectoria cartesiana generada (Altura = " << z[0] << " mm)";
    plt::title(title.str());

    plt::xlabel("X");
    plt::ylabel("Y");

    // Ajustar el tamaño de los ejes para que tengan la misma escala
    plt::axis("equal");

    // Generar la leyenda
    plt::legend();

    if (!filename.empty()) {
        plt::save(filename);
        std::cout << "Gráfico guardado en: " << filename << std::endl;
    }
    else {
        std::cout << "Mostrando el gráfico " << filename << std::endl;
        plt::show();
    }
    plt::close();

    return true;
}

