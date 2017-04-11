const int TRIG_PIN = 8;
const int ECHO_PIN = 9;

void setup() {
    Serial.begin(9600);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void loop() {
    long dist, duration;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    /*distanceCm = duration / 29.1 / 2;*/
    if (duration <= 0) {
        Serial.println(-1);
    } else {
        dist = 0.170145 * duration;
        Serial.println(dist);
    }
    delay(20);
}
