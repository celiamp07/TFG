# define STEP 4
# define DIR 5
const float resolution = 1.8;

void setup() {
  Serial.begin(9600); // Inicializalacomunicaci´onseriea9600bps
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
}

void loop() {
  // Sihayalg´unmensajedisponibleporelBUSdeserie,seejecuta
  if (Serial.available() > 0) {

    // LeeelmensajeenviadodesdePython
    String mensaje = Serial.readStringUntil('\n');
    Serial.print("Mensaje recibido: ");
    Serial.println(mensaje);

    // Inicializalasvariablesparaalmacenarlosgradosytiempodegiro
    int angle = 0;
    long turning_time = 0;

    // Mapeaelmensajeparaextraerlosgradosyeltiempodegiro
    int indexG = mensaje.indexOf('G');
    if (indexG != -1) {
      angle = mensaje.substring(indexG + 1, mensaje.indexOf('T')).toInt();
    }

    int indexT = mensaje.indexOf('T');
    if (indexT != -1) {
      turning_time = mensaje.substring(indexT + 1).toInt();
    }

    // Seenv´ıaunresumendelosdatosrecibidos
    Serial.print("Grados recibidos: ");
    Serial.println(angle);
    Serial.print("Tiempo degirorecibido: ");
    Serial.print(turning_time);
    Serial.println(" ms");

    // Conlosdatosrecibidosseprocedearealizarelgirodemotor
    move(angle, turning_time);
  }
}

void move(int angle, long turning_time) {
  // Activamos sentido horario fijo
  digitalWrite(DIR, LOW); 

  // Calculamos los pasos necesarios (180 / 1.8 = 100 pasos)
  int steps = abs(angle) / resolution;
  Serial.print("Se ejecutaran ");
  Serial.print(steps);
  Serial.println(" pasos de motor");

  // === CÁLCULO SEGURO EN MILISEGUNDOS CON DECIMALES ===
  // 1. Calculamos cuánto dura cada medio paso en milisegundos enteros
  float tiempo_medio_paso_ms = ((float)turning_time * resolution) / (2.0 * (float)abs(angle));
  
  // 2. Separamos la parte entera (ms) y la parte decimal (us)
  long ms_enteros = (long)tiempo_medio_paso_ms; 
  long us_restantes = (long)((tiempo_medio_paso_ms - (float)ms_enteros) * 1000.0);

  Serial.print("Delay combinado: ");
  Serial.print(ms_enteros);
  Serial.print(" ms y ");
  Serial.print(us_restantes);
  Serial.println(" microsegundos.");

  // Bucle de movimiento mixto (Ultra preciso y sin límites de desbordamiento)
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP, HIGH); 
    delay(ms_enteros);               // Espera la parte gorda en milisegundos (ej: 235 ms)
    delayMicroseconds(us_restantes); // Ajusta el pico de decimales pequeño (ej: 615 us)
    
    digitalWrite(STEP, LOW);  
    delay(ms_enteros);               
    delayMicroseconds(us_restantes); 
  }
  delay(2000); 
}