#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <string>
#include <iostream>
#include <matplotlibcpp.h>

// Mantenemos solo los includes de tipos que la clase necesita para existir
#include <abb_libegm/egm_trajectory_interface.h> 
#include "trajectory_generator.h"

struct DHParameters {
    double a;
    double alpha;
    double d;
    double theta;
};

struct JointLimits {
    double min;
    double max;
};

class Robot {
    public:
        //Constructores 
        Robot();
        Robot(const std::string& RobotFile);

        // Función para manejar la ejecución del movimiento del robot
        bool handleExecute(double speed_, double radius_);
        bool Simultaneo(abb::egm::EGMTrajectoryInterface& egm_interface, boost::asio::serial_port& serial,int angle, int turning_time);
        bool Secuencial(abb::egm::EGMTrajectoryInterface& egm_interface, boost::asio::serial_port& serial,int angle, int turning_time);

        // Función para manejar la resolución de la cinemática inversa con KDL
        bool handleSolveIK(const std::vector<Point>& points, TrajectoryGenerator MyTrajectory, bool debug);

        // Función para manejar el ploteo de las articulaciones
        bool handlePlotJoints(TrajectoryGenerator MyTrajectory,bool debug, const std::string& filename = "");
        void loadDataPlotJoints(const std::string& filename, std::vector<std::vector<double>>& datos, std::vector<double>& flags, bool debug);
        //void verificarLimitesArticulares(std::vector<std::vector<double>>& datos, const std::vector<JointLimits>& joint_limits);

    private: 
        std::vector<DHParameters> dh_params;
        std::vector<JointLimits> joint_limits;
        std::vector<std::vector<double>> jointTrajectory;
        void parseDHParameters(const std::string& filename);
        void parseJointLimits(const std::string& filename);
        abb::egm::wrapper::trajectory::TrajectoryGoal trajectory;

};

#endif // ROBOT_H

