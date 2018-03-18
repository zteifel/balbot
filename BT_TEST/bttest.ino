#include <SoftwareSerial.h>
SoftwareSerial BT(3,2);

void setup()
{
  BT.begin(9600);
  Serial.begin(115200);
  BT.println("Hello from Arduino");
}

String input;

void loop()
{
    if (BT.available()) {
        input = (String) BT.readString();
        Serial.print(input);
        input.trim();
        if (input == "de") {
            BT.println("funka");
        }
        BT.println(input);
    }
}
