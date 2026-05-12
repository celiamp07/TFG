#include "robot.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iomanip>
#include <thread>

// Includes de KDL (Movidos del .h al .cpp para mejorar la compilación)
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// Includes de ABB y Comunicaciones
#include <abb_libegm/egm_trajectory_interface.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

// Includes de YARP y Reconstrucción
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <SceneReconstructionIDL.h>

namespace plt = matplotlibcpp; // El alias se queda en el .cpp

void Robot::parseDHParameters(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);
    for (const auto& param : config["dh_parameters"]) {
        DHParameters dh;
        dh.a = param["a"].as<double>();
        dh.alpha = param["alpha"].as<double>();
        dh.d = param["d"].as<double>();
        dh.theta = param["theta"].as<double>();
        this->dh_params.push_back(dh);
    }
}

void Robot::parseJointLimits(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);
    for (const auto& node : config["joint_limits"]) {
        JointLimits limits;
        limits.min = node["min"].as<double>();
        limits.max = node["max"].as<double>();
        this->joint_limits.push_back(limits);
    }
}

//Constructor Vacío
Robot::Robot(){
    DHParameters dh;
        dh.a = 0;
        dh.alpha = 0;
        dh.d = 0;
        dh.theta = 0;
        this->dh_params.push_back(dh);
    JointLimits limits;
        limits.min = 0;
        limits.max = 0;
        this->joint_limits.push_back(limits);
    
}

//Constructor
Robot::Robot(const std::string& RobotFile){
    /*parseDHParameters(dhFile);
    parseJointLimits(limitsFile);*/
    parseDHParameters(RobotFile);
    parseJointLimits(RobotFile);
}

//Función que conecta con el robot
bool Robot::handleExecute() {
    std::cout << "Presione Enter para continuar y mover el robot al primer punto de la trayectoria..." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Iniciar YARP server
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::cerr << "No se pudo conectar al servidor de YARP\n";
        
    }

    // Abrir un puerto y conectarse a servicio externo
    yarp::os::RpcClient rpc;

    if (!rpc.open("/trajectory_generator_node/rpc:c") || !yarp::os::Network::connect(rpc.getName(), "/sceneReconstruction/rpc:s"))
    {
        std::cerr<< "No se pudo establecer la conexión con el servidor de recostrucción\n";
       
    }

    roboticslab::SceneReconstructionIDL sceneReconstruction;
    sceneReconstruction.yarp().attachAsClient(rpc);

    if (jointTrajectory.empty()) {
        std::cerr << "Error: No hay trayectoria calculada en memoria.\n";
        return false;
    }

    // Comunicación EGM para mover al primer punto
    // Componentes de Boost para manejar la comunicación asíncrona
    boost::asio::io_service io_service;
    boost::thread_group thread_group;

    // Configura la interfaz EGM para comunicarse con el robot 
    abb::egm::EGMTrajectoryInterface egm_interface(io_service, 6510);// Asignar  aquí el número de puerto correcto

    // Verifica que la interfaz de EGM se haya inicializado correctamente
    if (!egm_interface.isInitialized()) {
        std::cerr << "La inicialización de la interfaz EGM ha fallado\n";
        return false;
    }
    else {
        std::cout << "La interfaz EGM se ha inicializado correctamente\n";
    };
    
    // Crea un hilo para ejecutar io_service
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

    // Verifica que el robot esté conectado antes de enviar la trayectoria 
    while (!egm_interface.isConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));// Si no está conectado, el hilo descansa durante 500 ms antes de volver a verificar
    }

    // Mover al primer punto
    std::vector<double> first_joint_pos = jointTrajectory[0];
    abb::egm::wrapper::trajectory::TrajectoryGoal initial_trajectory;
    abb::egm::wrapper::trajectory::PointGoal* initial_point = initial_trajectory.add_points();

    for (size_t i = 0; i < first_joint_pos.size(); ++i) {
        initial_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(first_joint_pos[i]);
    }
    initial_point->set_duration(5.0); // Asignar una duración para alcanzar el primer punto

    std::cout << "Moviendo al primer punto...\n";
    egm_interface.addTrajectory(initial_trajectory);

    // Espera a que la ejecución de la trayectoria termine 
    abb::egm::wrapper::trajectory::ExecutionProgress execution_progress;
    bool wait = true;
    while (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (egm_interface.retrieveExecutionProgress(&execution_progress)) {
            wait = execution_progress.goal_active();// goal_active() devuelve true si la trayectoria aún está en curso, y false si ha terminado. Esto actualiza la variable wait, permitiendo que el bucle finalice cuando la ejecución de la trayectoria haya terminado
        }
    }
    std::cout << "Robot alcanzó el primer punto.\n";
     // Crear un hilo para ejecutar io_service
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    // Verifica que el robot esté conectado antes de enviar la trayectoria
    while (!egm_interface.isConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Enviando trayectoria circular...\n";
    
   // Enviamos el objeto 'trajectory'
    if (this->trajectory.points_size() > 0) {
        egm_interface.addTrajectory(this->trajectory);
    } else {
        std::cerr << "Error: La trayectoria circular está vacía en memoria.\n";
        return false;
    }

    // Activamos la cámara justo antes de empezar el movimiento circular
    sceneReconstruction.resume(); 
    
    wait = true;
    while (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (egm_interface.retrieveExecutionProgress(&execution_progress)) {
            wait = execution_progress.goal_active();
        }
    }

    std::cout << "Trayectoria finalizada.\n";
    
    // Pausamos la reconstrucción al terminar
    sceneReconstruction.pause(); 

    // APAGADO LIMPIO
    io_service.stop();
    thread_group.join_all();
   
    return true;
}

//Función que calcula la cinemática inversa
bool Robot::handleSolveIK(const std::vector<Point>& points, TrajectoryGenerator MyTrajectory, bool debug) {
    std::cout << "Presione Enter para continuar y calcular la cinemática inversa..." << std::endl;
    std::cin.get();

    // Configuración de la cadena cinemática (KDL)
    KDL::Chain chain;
    for (const auto& dh : dh_params) {
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh.a, dh.alpha * KDL::deg2rad, dh.d, dh.theta * KDL::deg2rad)));
    }
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0,0,30))));
    //chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Rotation::RPY(180.0*M_PI / 2.0, M_PI / 2.0, 0.0), KDL::Vector(0.0, 0.0, 30.0))));
    
    // Solver de cinemática directa
    KDL::ChainFkSolverPos_recursive FKSolverPos(chain);

    std::vector<double> initial_joints=MyTrajectory.getInitialJointPositions();
    
    // Posiciones articulares iniciales
    if (initial_joints.size() != 6) {
        std::cerr << "Error: Las posiciones iniciales de las articulaciones deben contener 6 valores.\n";
        return false;
    }

    //Posición inicial en coordenadas articulares
    KDL::JntArray q(6);
    for (size_t i = 0; i < 6; ++i) {
        q(i) = initial_joints[i] * KDL::deg2rad;
    }

    if (debug) {
        KDL::Frame H_0_6;
        FKSolverPos.JntToCart(q, H_0_6);
        std::cout << "Número de articulaciones: " << chain.getNrOfJoints() << std::endl; //Muesta por pantalla el número de articulaciones en la cadena cinemática
        //Muestra por pantalla las coordenadas x, y, z de la posición del extremo del robot
        std::cout << "Posición cartesiana inicial del robot:" << std::endl;
        std::cout << "Posición del extremo (x): " << H_0_6.p.x() << std::endl;
        std::cout << "Posición del extremo (y): " << H_0_6.p.y() << std::endl;
        std::cout << "Posición del extremo (z): " << H_0_6.p.z() << std::endl;

        // Muestra por pantalla las coordenadas articulares del extremo del robot
        std::cout << "Posición articular inicial del robot:" << std::endl;
        for (unsigned int i = 0; i < q.rows(); ++i) {
            std::cout << "Articulación " << i + 1 << ": " << q(i) * KDL::rad2deg << " grados" << std::endl;
        }
    }
   //Límites de las articulaciones extraídos del archivo YAML
    KDL::JntArray q_min(6);
    KDL::JntArray q_max(6);

    // Asignar los limites a las 6 articulaciones
    for (size_t i = 0; i < 6; ++i) {
        q_min(i) = joint_limits[i].min * KDL::deg2rad;
        q_max(i) = joint_limits[i].max * KDL::deg2rad;

        if (debug) {
            // Imprimir los límites para cada articulación
            std::cout << "Articulación " << i + 1 << ": ";
            std::cout << "Min: " << joint_limits[i].min << " grados, ";
            std::cout << "Max: " << joint_limits[i].max << " grados" << std::endl;
        }
    }

    //Solver de cinemática diferencial inversa
    KDL::ChainIkSolverVel_pinv IKSolverVel(chain);

    // Solver de cinemática inversa
    KDL::ChainIkSolverPos_NR_JL IKSolverPos(chain, q_min, q_max, FKSolverPos, IKSolverVel, 200000, 0.001);

    //  Limpieza de los atributos de la clase antes de empezar
    trajectory.Clear();       // El objeto TrajectoryGoal de la clase
    jointTrajectory.clear();  // El vector de vectores de la clase

    if (points.empty()) {
        std::cerr << "No hay puntos de trayectoria en memoria." << std::endl;
        return false;
    }

    double point_duration = MyTrajectory.calculatePointDuration(); // Ajustar según necesidad
    std::vector<double> min_values(6, std::numeric_limits<double>::max());
    std::vector<double> max_values(6, std::numeric_limits<double>::lowest());
    bool all_points_reachable = true;
    KDL::JntArray temp = q; 

    // Guardamos datos de la trayectoria
    std::vector<double> centro = MyTrajectory.getCentroid();
    double start_rad = MyTrajectory.getStartPosition() * KDL::deg2rad;
    double inclinacion_rad = MyTrajectory.getInclination() * KDL::deg2rad;

    std::cout << "Verificando trayectoria en memoria...\n" << std::endl;

    // Bucle de cálculo
    for (const auto& point : points) {
        // Añadimos punto al atributo 'trajectory' de la clase
        abb::egm::wrapper::trajectory::PointGoal* traj_point = trajectory.add_points();

        double angle = start_rad - atan2(point.y - centro[1], point.x - centro[0]);

        // Verificar cada punto de la trayectoria 
        KDL::Frame desiredPose;
        desiredPose.p = KDL::Vector(point.x, point.y, point.z);
        // Ajustar la rotación usando la secuencia de Euler YZY
        desiredPose.M = KDL::Rotation::RotY(180 * KDL::deg2rad) * KDL::Rotation::RotZ(angle) * KDL::Rotation::RotY(inclinacion_rad);

        KDL::JntArray result(6);

        int ret = IKSolverPos.CartToJnt(temp, desiredPose, result);
        
        int error_indicator = 0;

        for (size_t i = 0; i < 6; ++i) {
            double joint_angle_deg = result(i) * KDL::rad2deg;
            min_values[i] = std::min(min_values[i], joint_angle_deg);
            max_values[i] = std::max(max_values[i], joint_angle_deg);
        } 

        if (ret != 0) {
            bool exceeds_limits = false;
            for (size_t i = 0; i < 6; ++i) {
                if (result(i) <= q_min(i) || result(i) >= q_max(i)) {
                    exceeds_limits = true;
                    if (debug) {
                        std::cerr << "Error: La articulación " << i + 1 << " excede los límites. Valor: "
                                  << result(i) * KDL::rad2deg << " grados, Límite mínimo: "
                                  << q_min(i) * KDL::rad2deg << " grados, Límite máximo: "
                                  << q_max(i) * KDL::rad2deg << " grados.\n";
                    }
                }
            }

            if (exceeds_limits) {
                if (ret == KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED) {
                    std::cerr << "Punto inalcanzable en la trayectoria: ("
                              << point.x << ", " << point.y << ", " << point.z << ")\n";
                    all_points_reachable = false;
                    error_indicator = 1;
                }
            }
        }

        // Configurar puntos ABB
        std::vector<double> current_joints_deg;
        for (unsigned int i = 0; i < result.rows(); ++i) {
            double val = result(i) * KDL::rad2deg;
            traj_point->mutable_robot()->mutable_joints()->mutable_position()->add_values(val);
            current_joints_deg.push_back(val);
        }
        
        traj_point->set_duration(point_duration);
        jointTrajectory.push_back(current_joints_deg);

        if (debug) {
            std::cout << "Punto (" << point.x << ", " << point.y << ", " << point.z << ") OK." << std::endl;
        }

        temp = result;
    }

   
    // Resultados finales
    std::cout << "Resultados de la IK:\n";
    std::cout << "===================================\n";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << "Articulación " << i + 1 << ":\n";
        std::cout << "  Mínimo alcanzado: " << min_values[i] << "°\n";
        std::cout << "  Máximo alcanzado: " << max_values[i] << "°\n";
        std::cout << "  Límite mínimo: " << joint_limits[i].min << "°\n";
        std::cout << "  Límite máximo: " << joint_limits[i].max << "°\n";

        if (min_values[i] < joint_limits[i].min) {
            std::cout << "  Advertencia: El valor mínimo alcanzado excede el límite inferior\n";
        }
        if (max_values[i] > joint_limits[i].max) {
            std::cout << "  Advertencia: El valor máximo alcanzado excede el límite superior\n";
        }
    }
        
    if (all_points_reachable) {
        std::cout << "Todos los puntos generados son alcanzables.\n" << std::endl;
        return true;
    } else {
        std::cout << "La trayectoria contiene puntos inalcanzables. Ajusta los parámetros de entrada." << std::endl;
        return false;
    }
}


// Función para leer datos desde un archivo y devolver vectores de datos -> Para hacer el plot_joints
void Robot::loadDataPlotJoints(const std::string& filename, std::vector<std::vector<double>>& datos, std::vector<double>& flags, bool debug) {

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "No se pudo abrir el archivo: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double a1, a2, a3, a4, a5, a6, flag;
        ss >> a1 >> a2 >> a3 >> a4 >> a5 >> a6 >> flag;
        
        if (flag == 0) {  // Si el punto es alcanzable
            datos[0].push_back(a1);
            datos[1].push_back(a2);
            datos[2].push_back(a3);
            datos[3].push_back(a4);
            datos[4].push_back(a5);
            datos[5].push_back(a6);
        }
        else {  // Si el punto tiene error, agregar el valor capado
            datos[0].push_back(a1);
            datos[1].push_back(a2);
            datos[2].push_back(a3);
            datos[3].push_back(a4);
            datos[4].push_back(a5);
            datos[5].push_back(a6);
        }
        flags.push_back(flag);
    }

    file.close();

    // Mensaje de depuración
    if (debug) {
        std::cout << "Datos leídos del archivo:" << std::endl;
        for (size_t i = 0; i < datos[0].size(); ++i) {
            std::cout << datos[0][i] << " " << datos[1][i] << " " << datos[2][i] << " "
                << datos[3][i] << " " << datos[4][i] << " " << datos[5][i] << " " << flags[i] << std::endl;
        }
    }
    
}

// Función para trazar los puntos
bool Robot::handlePlotJoints(TrajectoryGenerator MyTrajectory, bool debug, const std::string& filename) {
    
    std::cout << "Presione Enter para continuar y ver el gráfico de los grados de las articulaciones..." << std::endl;
    std::cin.get();
    std::cout << "Generando gráfico..." << std::endl;
    
    //Gráficos de KDL
    std::vector<std::vector<double>> datos(6); // Vector de vectores para almacenar datos de cada articulación
    std::vector<double> flags;

    loadDataPlotJoints("kdl_joint_positions.txt", datos, flags, debug);

    double point_duration = MyTrajectory.calculatePointDuration();

    // Asumimos que todos los vectores de datos tienen la misma longitud
    int n = datos[0].size();
    if (debug) {
        std::cout << "Número de puntos leídos: " << n << std::endl;
    }
    
    std::vector<double> tiempo(n); //Vector de tiempo
    for (int i = 0; i < n; ++i) {
        tiempo[i] = i * point_duration;
    }

    if (debug) {
        // Imprimir los datos leídos
        std::cout << "Vector de tiempo:" << std::endl;
        for (const auto& t : tiempo) {
            std::cout << t << " ";
        }
        std::cout << std::endl;

        for (int i = 0; i < 6; ++i) {
            std::cout << "Datos del eje " << i + 1 << ":" << std::endl;
            for (const auto& d : datos[i]) {
                std::cout << d << " ";
            }
            std::cout << std::endl;
        }
    }

    // Añadir NaN en los puntos no alcanzables para las líneas
    std::vector<std::vector<double>> datos_con_nan = datos;
    for (int i = 0; i < n; ++i) {
        if (flags[i] != 0) {  // Si el punto es no alcanzable
            for (int j = 0; j < 6; ++j) {
                datos_con_nan[j][i] = std::numeric_limits<double>::quiet_NaN();  // Insertar NaN
            }
        }
    }

    plt::figure();

    // Dibujar datos alcanzables (líneas, ignorando NaN)
    plt::named_plot("q1", tiempo, datos_con_nan[0], "r-");
    plt::named_plot("q2", tiempo, datos_con_nan[1], "g-");
    plt::named_plot("q3", tiempo, datos_con_nan[2], "b-");
    plt::named_plot("q4", tiempo, datos_con_nan[3], "c-");
    plt::named_plot("q5", tiempo, datos_con_nan[4], "m-");
    plt::named_plot("q6", tiempo, datos_con_nan[5], "y-");

    // Dibujar puntos no alcanzables con scatter
    std::vector<double> tiempo_no_alcanzable;
    std::vector<std::vector<double>> datos_no_alcanzables(6);

    for (int i = 0; i < n; ++i) {
        if (flags[i] != 0) {  // Almacenamos puntos no alcanzables
            tiempo_no_alcanzable.push_back(tiempo[i]);
            for (int j = 0; j < 6; ++j) {
                datos_no_alcanzables[j].push_back(datos[j][i]);
            }
        }
    }

    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[0], 10.0, { {"color", "r"} }); // q1 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[1], 10.0, { {"color", "g"} }); // q2 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[2], 10.0, { {"color", "b"} }); // q3 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[3], 10.0, { {"color", "c"} }); // q4 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[4], 10.0, { {"color", "m"} }); // q5 capado
    plt::scatter(tiempo_no_alcanzable, datos_no_alcanzables[5], 10.0, { {"color", "y"} }); // q6 capado

    // Dibujar los límites articulares usando axhline y diferentes colores
    std::vector<std::string> limit_colors = { "r", "g", "b", "c", "m", "y" };  // Colores para los límites

    for (int i = 0; i < 6; ++i) {
        std::string color = limit_colors[i];
        // Dibujar límites mínimos y máximos
        plt::axhline(joint_limits[i].min, 0, tiempo.back(), { {"color", color}, {"linestyle", ":"}, {"linewidth", "0.5"} });
        plt::axhline(joint_limits[i].max, 0, tiempo.back(), { {"color", color}, {"linestyle", "-."}, {"linewidth", "0.5"} });

        // Convertir los valores de los límites a enteros para no mostrar decimales
        std::string min_label = std::to_string(static_cast<int>(joint_limits[i].min)) + "°";
        std::string max_label = std::to_string(static_cast<int>(joint_limits[i].max)) + "°";

        // Añadir anotaciones para los límites mínimo y máximo, utilizando plt::text para posicionar el texto
        plt::text(tiempo.back() * 1, joint_limits[i].min, min_label);  // Solo posición y texto
        plt::text(tiempo.back() * 1, joint_limits[i].max, max_label);  // Solo posición y texto

        // Calcular el valor mínimo y máximo alcanzado por la articulación
        double min_value = *std::min_element(datos[i].begin(), datos[i].end());
        double max_value = *std::max_element(datos[i].begin(), datos[i].end());

        // Encontrar los tiempos en los que se alcanzan el valor mínimo y máximo
        int min_index = std::distance(datos[i].begin(), std::min_element(datos[i].begin(), datos[i].end()));
        int max_index = std::distance(datos[i].begin(), std::max_element(datos[i].begin(), datos[i].end()));

        // Dibujar cruces en los puntos máximos y mínimos
        plt::scatter(std::vector<double>{tiempo[min_index]}, std::vector<double>{min_value}, 20, { {"color", color}, {"marker", "x"} });
        plt::scatter(std::vector<double>{tiempo[max_index]}, std::vector<double>{max_value}, 20, { {"color", color}, {"marker", "s"} });
    }

    // Crear entradas personalizadas en la leyenda para explicar los diferentes estilos de líneas
    plt::named_plot("Límite máximo", std::vector<double>{}, std::vector<double>{}, "k-.");
    plt::named_plot("Límite mínimo", std::vector<double>{}, std::vector<double>{}, "k:");
    plt::named_plot("Puntos inalcanzables", std::vector<double>{}, std::vector<double>{}, "k.");
    plt::named_plot("Valores máximos", std::vector<double>{}, std::vector<double>{}, "ks");
    plt::named_plot("Valores mínimos", std::vector<double>{}, std::vector<double>{}, "kx");

    plt::xlabel("Tiempo (s)");
    plt::ylabel("Grados");
    plt::title("Evolución temporal de las trayectorias articulares");
    
    // Crear la leyenda ajustando el tamaño y dividiéndola en dos columnas
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
