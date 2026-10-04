#define HOME1_PIN 25
#define HOME2_PIN 26

void setup()
{
    Serial.begin(115200);

    pinMode(HOME1_PIN, INPUT_PULLUP);
    pinMode(HOME2_PIN, INPUT_PULLUP);
}

void loop()
{
    Serial.print("LEFT HOME: ");
    Serial.print(digitalRead(HOME1_PIN));

    Serial.print("    RIGHT HOME: ");
    Serial.println(digitalRead(HOME2_PIN));

    delay(200);
}