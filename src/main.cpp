// Clases
#include "trajectory_generator.h" 
#include "robot.h"

// Librerías
#include <boost/program_options.hpp>
#include <matplotlibcpp.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

namespace plt = matplotlibcpp;
namespace po = boost::program_options;

// Sobrecarga de operator<< para imprimir Point
std::ostream& operator<<(std::ostream& os, const Point& point) {
    os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return os;
}

// Función para cargar y mostrar los datos del robot desde un archivo YAML
void loadRobotData(const std::string& filename) {
    try {
        YAML::Node robot = YAML::LoadFile(filename);
        std::cout << "Datos del robot leídos del archivo: " << filename << std::endl;
        std::cout << "\n===================================\n";

        // Mostrar el nombre del robot
        std::cout << "Robot:\n";
        std::cout << "  " << robot["robot"].as<std::string>() << std::endl;

        std::cout << "===================================\n";

        // Mostrar los parámetros DH en formato tabla
        std::cout << "DH Parameters:\n";
        std::cout << std::setw(8) << "a" << std::setw(8) << "alpha" << std::setw(12) << "d" << std::setw(8) << "theta" << "\n";
        std::cout << "  ----------------------------------------\n";
        for (const auto& param : robot["dh_parameters"]) {
            std::cout << std::setw(8) << param["a"].as<double>()
                << std::setw(8) << param["alpha"].as<int>()
                << std::setw(12) << param["d"].as<double>()
                << std::setw(8) << param["theta"].as<int>() << std::endl;
        }

        std::cout << "===================================\n";

        // Mostrar los límites de las articulaciones en formato tabla
        std::cout << "Joint Limits:\n";
        std::cout << std::setw(8) << "min" << std::setw(8) << "max" << "\n";
        std::cout << "  ------------\n";
        for (const auto& limit : robot["joint_limits"]) {
            std::cout << std::setw(8) << limit["min"].as<double>()
                << std::setw(8) << limit["max"].as<double>() << std::endl;
        }

        std::cout << "===================================\n";
    }
    catch (const YAML::Exception& e) {
        std::cerr << "Error al leer el archivo YAML: " << e.what() << std::endl;
    }
}

//InputParameters input_params; // Variable para almacenar los datos de entrada parseados del archivo YAML

int main(int argc, char** argv) {
    Robot MyRobot;
    TrajectoryGenerator MyTrajectory;
    try {
        po::options_description desc("Opciones permitidas");
        desc.add_options()
            ("help", "Mostrar mensaje de ayuda\n")
            ("debug", "Habilitar el modo debug")
            ("generate", po::value<std::string>(), "Generar parámetros\n")
            ("plot", po::value<std::string>()->implicit_value(""), "Gráfico de puntos\n")
            ("robot", po::value<std::string>(), "Archivo YAML del robot\n")
            ("solveIK", "Ejecutar cinemática inversa\n")
            ("plot_joints", po::value<std::string>()->implicit_value(""), "Gráfico de articulaciones\n")
            ("execute", "Ejecutar movimientos\n");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        bool debug = vm.count("debug");

        if (vm.count("help") || vm.empty()) {
            std::cout << desc << std::endl;
            return 1;
        }


        if (vm.count("generate")) {
            std::string gen = vm["generate"].as<std::string>();
            if (gen == "input") MyTrajectory.readFromConsole();
            else MyTrajectory.parseInputParameters(gen);
            MyTrajectory.handleGenerate(debug);
        } else if (vm.count("plot_joints") || vm.count("solveIK")) {
            // Carga por defecto para que input_params no esté vacío
            MyTrajectory.parseInputParameters("input_parameters.yml");
        }

        if (vm.count("plot")) MyTrajectory.handlePlot(vm["plot"].as<std::string>());

        // Sacamos la llamada de los condicionales para que se ejecute siempre que esté el flag
        if (vm.count("robot")) {
            std::string robot_file = vm["robot"].as<std::string>();
            MyRobot=Robot (robot_file); // Se carga siempre aquí
            
            // Si solo se quiere visualizar la info del robot (sin otras acciones)
            if (vm.size() == 1 || (vm.size() == 2 && debug)) {
                loadRobotData(robot_file);
            }
        }

        // --- SOLVE IK ---
        if (vm.count("solveIK")) {
            if (!vm.count("robot")) {
                std::cerr << "Error: --solveIK requiere --robot\n";
                return 1;
            }
            MyRobot.handleSolveIK(MyTrajectory.getGeneratedPoints(), MyTrajectory, debug);
        }

        // --- PLOT JOINTS ---
        if (vm.count("plot_joints")) {
            MyRobot.handlePlotJoints(MyTrajectory, debug, vm["plot_joints"].as<std::string>());
        }

        if (vm.count("execute")) MyRobot.handleExecute();       

    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}