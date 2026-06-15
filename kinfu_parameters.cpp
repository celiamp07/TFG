#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <iomanip>

using namespace std;
using namespace cv;

// --- CONFIGURACIÓN DE RUTAS ---
string INI_PATH = "/home/celia/vision/share/contexts/sceneReconstruction/sceneReconstruction-realsense2.ini";
string EJECUTABLE = "sceneReconstruction";
string COMANDO_LANZAR = EJECUTABLE + " --from " + INI_PATH + " &"; 
string COMANDO_MATAR = "pkill -f " + EJECUTABLE;

struct KinfuParams {
    int bilateral, icp, truncate, voxSize;
    int transX, transY, transZ;
    int dimX, dimY, dimZ;
};

// --- FUNCIONES DE SOPORTE ---
void extraerValores(string linea, vector<float>& out) {
    size_t start = linea.find_first_of("0123456789.-");
    if (start == string::npos) return;
    string limpia = linea.substr(start);
    for (char &c : limpia) if (c == '(' || c == ')') c = ' ';
    stringstream ss(limpia);
    float val;
    while (ss >> val) out.push_back(val);
}

KinfuParams cargarValoresDesdeIni() {
    KinfuParams p = {1, 10, 25, 2, 50, 50, 23, 100, 90, 90}; 
    ifstream file(INI_PATH);
    if (!file.is_open()) return p;
    string line;
    while (getline(file, line)) {
        vector<float> v;
        if (line.find("//") == 0) continue;
        if (line.find("bilateralSigmaDepth") != string::npos) { extraerValores(line, v); if(!v.empty()) p.bilateral = v[0]*100; }
        else if (line.find("icpDistThresh") != string::npos) { extraerValores(line, v); if(!v.empty()) p.icp = v[0]*1000; }
        else if (line.find("truncateThreshold") != string::npos) { extraerValores(line, v); if(!v.empty()) p.truncate = v[0]*10; }
        else if (line.find("voxelSize") != string::npos) { extraerValores(line, v); if(!v.empty()) p.voxSize = v[0]*1000; }
        else if (line.find("volumePoseTransl") != string::npos) { 
            extraerValores(line, v); 
            if(v.size() >= 3) { p.transX = (int)(v[0]*100 + 50); p.transY = (int)(v[1]*100 + 50); p.transZ = (int)(v[2]*100); }
        }
        else if (line.find("volumeDims") != string::npos) { 
            extraerValores(line, v); 
            if(v.size() >= 3) { p.dimX = v[0]; p.dimY = v[1]; p.dimZ = v[2]; }
        }
    }
    file.close();
    return p;
}

void guardarEnArchivo(KinfuParams p) {
    ifstream fileIn(INI_PATH);
    if (!fileIn.is_open()) return;
    string content = "", line;
    while (getline(fileIn, line)) {
        stringstream ss; ss << fixed << setprecision(6);
        if (line.find("//") == 0) { content += line + "\n"; continue; }
        if (line.find("bilateralSigmaDepth") != string::npos) { ss << p.bilateral/100.0; line = "bilateralSigmaDepth  " + ss.str(); }
        else if (line.find("icpDistThresh") != string::npos) { ss << p.icp/1000.0; line = "icpDistThresh        " + ss.str(); }
        else if (line.find("truncateThreshold") != string::npos) { ss << p.truncate/10.0; line = "truncateThreshold    " + ss.str(); }
        else if (line.find("voxelSize") != string::npos) { ss << p.voxSize/1000.0; line = "voxelSize            " + ss.str(); }
        else if (line.find("volumePoseTransl") != string::npos) 
            line = "volumePoseTransl     (" + to_string((p.transX-50)/100.0) + " " + to_string((p.transY-50)/100.0) + " " + to_string(p.transZ/100.0) + ")";
        else if (line.find("volumeDims") != string::npos) 
            line = "volumeDims           (" + to_string(p.dimX) + " " + to_string(p.dimY) + " " + to_string(p.dimZ) + ")";
        content += line + "\n";
    }
    fileIn.close();
    ofstream fileOut(INI_PATH);
    if (fileOut.is_open()) { fileOut << content; fileOut.close(); }
}

int main() {
    KinfuParams p = cargarValoresDesdeIni();
    namedWindow("Panel de Control Kinfu", WINDOW_NORMAL);
    resizeWindow("Panel de Control Kinfu", 600, 600);

    // Crear Sliders
    createTrackbar("Bilateral x100", "Panel de Control Kinfu", &p.bilateral, 15);
    createTrackbar("ICP x1000", "Panel de Control Kinfu", &p.icp, 20);
    createTrackbar("Trunc x10", "Panel de Control Kinfu", &p.truncate, 50);
    createTrackbar("Trans X", "Panel de Control Kinfu", &p.transX, 100);
    createTrackbar("Trans Y", "Panel de Control Kinfu", &p.transY, 100);
    createTrackbar("Trans Z", "Panel de Control Kinfu", &p.transZ, 100);
    createTrackbar("Dim X", "Panel de Control Kinfu", &p.dimX, 300);
    createTrackbar("Dim Y", "Panel de Control Kinfu", &p.dimY, 300);
    createTrackbar("Dim Z", "Panel de Control Kinfu", &p.dimZ, 300);

    bool ejecutando = false;
    string estado = "PARADO";
    Scalar colorEstado = Scalar(0,0,255);

    while (true) {
        Mat interfaz = Mat::zeros(300, 1200, CV_8UC3); // Fondo negro
        
        // Título de Estado centrado
        putText(interfaz, "ESTADO: " + estado, Point(450, 60), FONT_HERSHEY_DUPLEX, 1.2, colorEstado, 2);
        line(interfaz, Point(50, 85), Point(1150, 85), Scalar(150, 150, 150), 2);
        
        putText(interfaz, "CONTROLES:", Point(50, 130), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(200, 200, 200), 2);
        
        // Coordenada X fija para todos (100) para que estén alineados a la izquierda
        int posX = 100; 

        // R - Verde
        putText(interfaz, "[R] REINICIAR: Aplica cambios y lanza camara", 
                Point(posX, 170), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        
        // S - Azul (BGR: 255, 0, 0)
        putText(interfaz, "[S] GUARDAR: Escribe valores en el .ini de forma permanente", 
                Point(posX, 210), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
        
        // K - Naranja (BGR: 0, 165, 255)
        putText(interfaz, "[K] MATAR: Detiene el proceso actual", 
                Point(posX, 250), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 165, 255), 2);
        
        // ESC - Rojo (BGR: 0, 0, 255)
        putText(interfaz, "[ESC] SALIR: Cierra el panel de control", 
                Point(posX, 290), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);

        imshow("Panel de Control Kinfu", interfaz);

        char key = (char)waitKey(30);

        if (key == 27) { // ESC
            system(COMANDO_MATAR.c_str());
            break;
        }
        else if (key == 'r' || key == 'R') {
            estado = "REINICIANDO...";
            colorEstado = Scalar(0, 255, 255); // Amarillo mientras reinicia
            system(COMANDO_MATAR.c_str());
            guardarEnArchivo(p);
            system(COMANDO_LANZAR.c_str());
            estado = "EJECUTANDO";
            colorEstado = Scalar(0, 255, 0); // Verde
            ejecutando = true;
        }
        else if (key == 'k' || key == 'K') {
            system(COMANDO_MATAR.c_str());
            estado = "PARADO";
            colorEstado = Scalar(0, 0, 255); // Rojo
            ejecutando = false;
        }
        else if (key == 's' || key == 'S') {
            guardarEnArchivo(p);
            cout << "\n>>> PARAMETROS GUARDADOS <<<" << endl;
            estado = "GUARDADO OK";
            colorEstado = Scalar(255, 255, 0); // Cyan para el guardado
        }
    }
    return 0;
}
