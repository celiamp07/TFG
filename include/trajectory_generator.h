#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#define _USE_MATH_DEFINES
#include <vector>
#include <cmath>
#include <string>

// Definición de una estructura para representar un punto en 3D
struct Point {
    double x; // coordenada x
    double y; // coordenada y
    double z; // coordenada z
};

// Definición de la clase TrajectoryGenerator
class TrajectoryGenerator {
public:
    TrajectoryGenerator();
    TrajectoryGenerator(const std::vector<double>& centroid_, double radius_, double height_, double inclination_, int resolution_, double start_position_, double end_position_, double speed_); // Inicializa los parámetros de la circunferencia
    std::vector<Point> generateCircularTrajectory(); // Genera y devuelve un vector de puntos que representan una trayectoria circular en el espacio 3D
    double calculatePointDuration() const; // Calcula la duración para alcanzar cada punto
    // Función para leer los parámetros de entrada desde el teclado
    void readFromConsole();
    // Función para generar y visualizar la trayectoria
    std::vector <Point> handleGenerate(bool debug);
    // Función para manejar el ploteo de datos de la trayectoria
    bool handlePlot(const std::string& filename = "");
    bool loadDataPlot(const std::string& filename, std::vector<Point>& points);

    //GETS
    std::vector<double> getCentroid() const { return centroid; }
    double getRadius() const { return radius; }
    double getHeight() const { return height; }
    double getInclination() const { return inclination; }
    int getResolution() const { return resolution; }
    double getSpeed() const { return speed; }
    double getStartPosition() const { return start_position; }
    double getEndPosition() const { return end_position; }
    std::vector<double> getInitialJointPositions() const { return initial_joint_positions; }
    std::vector<Point> getGeneratedPoints() const { return generated_points; }

    void parseInputParameters(const std::string& filename);
private: //Almacena los parámetros de entrada

    std::vector<double> centroid {0.0, 0.0, 0.0};
    double radius {0.0};
    double height {0.0};
    double inclination {0.0};
    int resolution {0};
    double speed {0.0};
    double start_position {0.0}; // Ángulo inicial en grados
    double end_position {0.0}; // Ángulo final en grados
    std::vector<double> initial_joint_positions;

    std::vector<Point> generated_points;
};

#endif // TRAJECTORY_GENERATOR_H

