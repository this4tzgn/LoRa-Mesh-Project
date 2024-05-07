
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "uFire_SHT3x.h"
#include <SPIFFS.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include "RTClib.h"

#define DEFAULT_BAUD_RATE 115200

//COMMAND TOPICS
#define NEWLOCALADDRESS   "newlocaladr" // yeni local adres vermek için komut newlocaladr=yeniadres#
#define MESSAGE           "msg" // mesaj gönderme komutu msg=adres:mesaj#
#define DATABASE          "shwdb"// database listeleme komutu shwdb=address# address = -1 için tüm database'i listele
#define NEWTIME           "newtime"

using json = nlohmann::json;
json deviceDatabase; //database'i oluştur

uFire::SHT3x sht30;
RTC_DS3231 rtc;

// ekran ayarlarını yap
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define NUMFLAKES     10 // Number of snowflakes in the animation example


//define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2

#define DEVICE_COUNT 100 // ağda maximum kaç cihaz olacağını belirtir
int LOCALADR = 0x00;//default yerel adres
#define BROADCASTCHANNEL 0xFF // genel yayın kanalı
#define TARGETADR 0x01 

uint8_t lastInsertedAddress = -1; // data insert ederken son insert edilen adresi saklar
bool deviceCounter[101];// hangi adresteki cihazların adreste anlık kayıtlı olduğunu gösterir
unsigned long lastHarvestTime = 0;// veri toplama süresini ayarlamak için değişken
unsigned long lastMesherTime = 0;// ağa yerel bilgiler harici bilgileri basmak için kullanılır 
uint8_t meshQueue = 0; // adresleri sıraya sokmak için kullanılır. her gönderimde bir adresin verisi iletilir. aşırı yükleme yapılmaz
int insertctr = 0; // kaç verinin istert edildiğini gösterir
///fonksiyonlar:

void listenLoraNetwork(int packetSize);//lora ağını dinler ve verileri toplar
String splitter(String data, char separator, int index);//string dataları ayırmada kullanılır
void sendMessage(String outgoing,uint8_t localADR);//lora ağına mesaj yollamada kullanılır
void writeMessage2Screen(String message);// ekrana string yazdırmada kullanılır
void ChangeLocalADR(String input);// yerel adresi değiştirir
void SetUpDisplay();// ekran ayarlarını yapar
void SetUpLora();// lora ayarlarını yapar
String ShowDatabase(int address);//database'teki verileri listeler
void DataHarverster();// dataları toplar ve toplanan dataları database'e kaydeder
void LoraDataPackager(int address);//dataları lora üzerinden gönderecek formata çevirir ve gönderme işlemi yapar
void DataInserter(uint8_t address,String DataBlock);// gelen dataları lora gönderim formatından database formatına çevirir
void regularScreenTable();// sürekli gösterilen ekran bilgilerini yeniler
void messenger(int address, String message,int block);// boardcast kanalından gidecek olan mesajı uygun formata sokar adres;mesaj
void checkBroadcastChannel();// broadcast kanalında data var mı diye kontrol eder, eğer varsa gönderir, eğer hedef cihazın inbox bilgisi bu mesaja değiştiyse 255 kanalından datayı siler ve göndermeyi bırakır
void meshNetwork();//ağda kayıtlı olan kendisi dışındaki cihazların verilerini ağa geri belirli bir sıraya koyarak basar
String CompareTime(String timeBlock1,String timeBlock2);//String biçiminde olan iki farklı zaman değerini karşılaştırıp güncel olanı döndürür
void changeTime(uint8_t address,String NewTime);// database'teki yerel zaman değerini değiştirir

void setup() {
  LOCALADR = random(1,100); // cihaz açılışta rastgele bir adres alır
  randomSeed(analogRead(36));
  Serial.begin(DEFAULT_BAUD_RATE);// seri portu başlatır
  while (!Serial);
  Serial.println("ID: " + (String)LOCALADR);// Seri porta cihazın güncel adresini yaz
  //display settings
  SetUpDisplay();
  //setup LoRa transceiver module
  SetUpLora();
  
  // Flash hafızayı kontrol et
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // database'te cihaz sayısı kadar template oluşturulur
  for(int i=0;i<=DEVICE_COUNT;i++){
    deviceDatabase[i]["Address"]       = i;
    deviceDatabase[i]["time"]         = "1970-01-01 00:00:00";
    deviceDatabase[i]["latitude"]     = "0.0";
    deviceDatabase[i]["longitude"]    = "0.0";
    deviceDatabase[i]["temp&hum"]     = "0.0;0.0";
    deviceDatabase[i]["gyro_x_y_z"]   = "0.0;0.0;0.0";
    deviceDatabase[i]["inBox"]        = "";
    deviceDatabase[i]["sendBox"]      = "";
    deviceCounter[i] = false;
  }
  DateTime now;
  String date = (String)(now.year())+"-"+(String)(now.month()) +"-"+ (String)(now.day()) +" "+(String)(now.hour())+":"+(String)(now.minute())+":"+(String)(now.second());
  deviceDatabase[LOCALADR]["time"]         = date.c_str();// yerel datalara örnek bir zaman yüklenir
  // 5 adet broadcast channel oluşturulur
  deviceDatabase[BROADCASTCHANNEL][0]      = "";
  deviceDatabase[BROADCASTCHANNEL][1]      = "";
  deviceDatabase[BROADCASTCHANNEL][2]      = "";
  deviceDatabase[BROADCASTCHANNEL][3]      = "";
  deviceDatabase[BROADCASTCHANNEL][4]      = "";




  deviceCounter[LOCALADR] = true;// cihaz kendini ağa kaydeder.
}

void loop() {
  listenLoraNetwork(LoRa.parsePacket());// loradan gelen dataları dinle
  if(millis() - lastHarvestTime > random(1000,2500)){// belirli aralıklarla dataları topla ve ilet 
    DataHarverster();
    regularScreenTable();
    lastHarvestTime = millis();
  }
  
  meshNetwork(); // ağa veri bas

  //Seri portu dinle
  while(Serial.available()){
    String input = Serial.readStringUntil('#');
    Serial.println("Serial message: " + input);

    if(splitter(input,'=',0) == NEWLOCALADDRESS){// yeni adres verildiyse onu güncellemek için fonksiyonu çağır
      ChangeLocalADR(input);
    }
    if(splitter(input,'=',0) == MESSAGE){// mesaj geldiyse onu broadcast kanallarına ilet
      int address = splitter(splitter(input,'=',1),':',0).toInt();
      String message = splitter(splitter(input,'=',1),':',1);
      messenger(address,message,0);// şimdilik sadece kanal 0 ileride sıraya konacak
    }
    if(splitter(input,'=',0) == DATABASE){// istenilen adresteki cihaz bilgilerini listele
      ShowDatabase(splitter(input,'=',1).toInt());  
    }
    if(splitter(input,'=',0) == NEWTIME){// istenen zamanı güncelle
      changeTime(LOCALADR,splitter(input,'=',1)); // zamanı güncelle  
    } 
  }
}

void listenLoraNetwork(int packetSize) {
    if (packetSize == 0) return;
    // gelen verileri okumaya başla:
    
    int senderADR1 = LoRa.read();          // alıcı adresi
    int senderADR2 = LoRa.read();            // gönderici adresi
    int incomingLength = LoRa.read();    // gelen mesajın boyutu
    
    if(senderADR1 != senderADR2 || senderADR1 == LOCALADR) return; // adres doğrulaması yapılamazsa fonksiyonu sonlandır

    writeMessage2Screen("lora catch a packet from " + (String)senderADR1);// ekrana gelen adres bilgisi yazdır
    // mesaj doğrulama bölümü
    String incoming1 = "";
    String incoming2 = "";

    int i = incomingLength;
    while (i!=0) {
        incoming1 += (char)LoRa.read();
        i--;
    }
    i = incomingLength;
    while (i!=0) {
        incoming2 += (char)LoRa.read();
        i--;
    }
    if (incomingLength != incoming1.length() && incomingLength != incoming2.length())    return;       // mesajda kesinti var mı diye kontrol et??
    // mesaj doğrulama bölümü sonu
    if (incoming1 == incoming2){ 
      DataInserter(senderADR1,incoming1);// dataları database'e yerleştir
    }
}

void sendMessage(String outgoing,uint8_t localADR) {
      LoRa.beginPacket();                   // aktarım başlat
      LoRa.write(localADR);                 // local adresi ver
      LoRa.write(localADR);                 // doğrulama için ikinci local adres
      LoRa.write(outgoing.length());        // mesajın uzunluğu
      LoRa.print(outgoing);                 // mesajın gönderme işlemi
      LoRa.print(outgoing);                 // doğrulama paketi gönderimi
      LoRa.endPacket();                     // aktarımı kapat
  }

String splitter(String data, char separator, int index){
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length()-1;

    for(int i=0; i<=maxIndex && found<=index; i++){
        if(data.charAt(i)==separator || i==maxIndex){
            found++;
            strIndex[0] = strIndex[1]+1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void writeMessage2Screen(String message) {
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(message);
  display.display();
}
void ChangeLocalADR(String input){
  if(splitter(input,'=',1).toInt() > 0 && splitter(input,'=',1).toInt() < 255){// verilen adres değerli geçerli bir aralıkta mı
    LOCALADR = splitter(input,'=',1).toInt();// öyle ise eşitle
    writeMessage2Screen("Device Address changed to : " + (String)LOCALADR);// ekrana bildirim gönder
  }
  else{
    writeMessage2Screen("Device Address is not suitable with network, current address is : " + (String)LOCALADR);//değişmedi ise de bvildirim gönder
  }
}
void SetUpDisplay(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { 
    Serial.println(F("Display Couldn't Started !"));
  }
  display.display();
  delay(2000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();
  writeMessage2Screen("LoRa Mesh Project");
  Serial.println("Display Started!");
}
void SetUpLora(){
  LoRa.setPins(ss, rst, dio0);
  byte c_try = 0;
  while (!LoRa.begin(433E6) && c_try < 5) {
    Serial.println("Trying again for run the LoRa..");
    c_try++;
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Started!");
}
String ShowDatabase(int address){
  if(address == -1){// eğer -1 değeri ilse istenirse tüm database'i listele
    std::string serializedObjectPretty = deviceDatabase.dump(3);
    Serial.println("\n\nDevice Database:");
    Serial.println(serializedObjectPretty.c_str());
    return serializedObjectPretty.c_str();
  }
  else if(address > 0 && address < 101){// istene adrese göre listeleme yap
    std::string serializedObjectPretty = deviceDatabase[address].dump(3);
    Serial.println("\n\nDevice Database:");
    Serial.println(serializedObjectPretty.c_str());
    return serializedObjectPretty.c_str();
  }    
}
// dataları topla vce database'e yerleştir
void DataHarverster(){
  DateTime now;
  String date = (String)(now.year())+"-"+(String)(now.month()) +"-"+ (String)(now.day()) +" "+(String)(now.hour())+":"+(String)(now.minute())+":"+(String)(now.second());
  String tempandhum = (String)(random(-10000.0,10000.0)/100.0) + ";" + (String)(random(0.0,10000.0)/100.0);
  String gyros = (String)(random(-10.0,10.0)/10.0) + ";" + (String)(random(-10.0,10.0)/10.0) + ";" + (String)(random(-10.0,10.0)/10.0);
  

  //deviceDatabase[LOCALADR]["time"] = date.c_str();
  deviceDatabase[LOCALADR]["temp&hum"] = tempandhum.c_str();
  deviceDatabase[LOCALADR]["Address"]       = LOCALADR;
  deviceDatabase[LOCALADR]["latitude"]     = "36.929591280415";
  deviceDatabase[LOCALADR]["longitude"]    = "30.65711958584675";
  deviceDatabase[LOCALADR]["gyro_x_y_z"]   = gyros.c_str();
  //deviceDatabase[LOCALADR]["inBox"]        = "inbox";
  deviceDatabase[LOCALADR]["sendBox"]      = "sendbox";

  LoraDataPackager(LOCALADR);// en son olarak toplanan dataları lora formatına paketlemesi içöin fonksiyona ver
}
void LoraDataPackager(int address){
  auto localTime      =   deviceDatabase[address]["time"].get<std::string>();
  auto tempNhum       =   deviceDatabase[address]["temp&hum"].get<std::string>();
  auto localaddress   =   deviceDatabase[address]["Address"].get<int>(); 
  auto latitude       =   deviceDatabase[address]["latitude"].get<std::string>();
  auto longitude      =   deviceDatabase[address]["longitude"].get<std::string>(); 
  auto gyro_x_y_z     =   deviceDatabase[address]["gyro_x_y_z"].get<std::string>(); 
  auto inBox          =   deviceDatabase[address]["inBox"].get<std::string>(); 
  auto sendBox        =   deviceDatabase[address]["sendBox"].get<std::string>();

  String Package = "";
  Package += localTime.c_str();
  Package += "$";
  Package += tempNhum.c_str();
  Package += "$";
  Package += (String)localaddress;
  Package += "$";
  Package += latitude.c_str();
  Package += "$";
  Package += longitude.c_str();
  Package += "$";
  Package += gyro_x_y_z.c_str();
  Package += "$";
  Package += inBox.c_str();
  Package += "$";
  Package += sendBox.c_str();
  Package += "$";
  
  sendMessage(Package,address);//hazırlanan paketi gönder
}
void DataInserter(uint8_t address,String DataBlock){
  if(address == BROADCASTCHANNEL){// eğer broadcast kanalından geliyorsa farklı bir şekilde insert et
    for(int i=0;i<5;i++){
      if(splitter(splitter(DataBlock,'$',i),';',0).toInt() == LOCALADR){ // adres yerel adresle uyuşuyor mu?
        // eğer uyuşuyorsa inBox'a o değeri yerleştir
        deviceDatabase[LOCALADR]["inBox"] = splitter(splitter(DataBlock,'$',i),';',1).c_str();   
      }
    }
  }
  else if(
    (String)(deviceDatabase[address]["time"].get<std::string>().c_str())  // gelen data bloğunun zamanının mı yoksa yerelde kayıtlı bloğun zamanının mı güncel olduğunu kontrol et
    != 
    CompareTime((String)(deviceDatabase[address]["time"].get<std::string>().c_str()),splitter(DataBlock,'$',0))
    &&
    (address >= 0 || address <= DEVICE_COUNT) // gelen adres geçerli adres aralığında mı diye kontrol et
    ){
    Serial.println(CompareTime((String)(deviceDatabase[address]["time"].get<std::string>().c_str()),splitter(DataBlock,'$',0)));
    deviceDatabase[address]["time"]         = splitter(DataBlock,'$',0).c_str();
    deviceDatabase[address]["temp&hum"]     = splitter(DataBlock,'$',1).c_str();
    deviceDatabase[address]["Address"]      = splitter(DataBlock,'$',2).toInt();
    deviceDatabase[address]["latitude"]     = splitter(DataBlock,'$',3).c_str();
    deviceDatabase[address]["longitude"]    = splitter(DataBlock,'$',4).c_str();
    deviceDatabase[address]["gyro_x_y_z"]   = splitter(DataBlock,'$',5).c_str();
    deviceDatabase[address]["inBox"]        = splitter(DataBlock,'$',6).c_str();
    deviceDatabase[address]["sendBox"]      = splitter(DataBlock,'$',7).c_str();
    //Serial.println("Data inserter inserted the datas came from " + (String)address);
    lastInsertedAddress = address;
    deviceCounter[address] = true;
    insertctr ++;
  }
  
}
void regularScreenTable(){
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println("The LoRa Mesh");
  display.println("Local: "+(String)LOCALADR);
  display.println("inserted: "+(String)lastInsertedAddress);
  
  int count = 0;
    for (int i = 0; i < sizeof(deviceCounter) / sizeof(deviceCounter[0]); ++i)
        count += deviceCounter[i];
  display.println("devices: "+(String)count);
  display.println("message: "+(String)deviceDatabase[LOCALADR]["inBox"].get<std::string>().c_str()); 
  display.println("insert ctr: "+(String)insertctr);
  display.display();
}
void messenger(int address, String message,int block){
  deviceDatabase[BROADCASTCHANNEL][block] = ((String)address + ";" + message).c_str();
}

void checkBroadcastChannel(){
  String temp = ""; 
  for(int i = 0;i<5;i++){
    if((String)deviceDatabase[splitter(deviceDatabase[BROADCASTCHANNEL][i].get<std::string>().c_str(),';',0).toInt()]["inBox"].get<std::string>().c_str() == splitter(deviceDatabase[BROADCASTCHANNEL][i].get<std::string>().c_str(),';',1)){ // eğer inBox'a mesaj ulaştıysa broadcast yayınını durdur bu cihaz için
      deviceDatabase[BROADCASTCHANNEL][i] = "";
    }
    if((String)(deviceDatabase[BROADCASTCHANNEL][i].get<std::string>().c_str()) != ""){
      temp += deviceDatabase[BROADCASTCHANNEL][i].get<std::string>().c_str();
      temp += '$';
    }  
  }
  if(temp != "") sendMessage(temp,BROADCASTCHANNEL);
}
void meshNetwork(){
  if(millis() - lastMesherTime > random(2000,3000)){
    checkBroadcastChannel();
    for(int i=meshQueue;i<100;i++){
      String tempDate = deviceDatabase[i]["time"].get<std::string>().c_str();
      if(tempDate != "1970-01-01 00:00:00" & i != LOCALADR ){// default tanımdan farklıysa ağa bas
        LoraDataPackager(i); // eğer ağda kayıtlı cihaz varsa onu da yayınla
        Serial.println((String)i + "adresinden gönderildi");
        if(meshQueue == 254) meshQueue = 0;
        else  meshQueue ++; // bir sonraki cihazı sıraya al
        break; // 1 cihazdan fazla gönderme tek döngüde
      }
    }
    lastMesherTime = millis();
  }
}
//zaman datasını string formattan alıp hangi zamanın daha güncel olduğunu kontrol eder.
String CompareTime(String timeBlock1,String timeBlock2){
  uint8_t year1,month1,day1,hour1,minute1,second1;
  uint8_t year2,month2,day2,hour2,minute2,second2;
  year1   = splitter(splitter(timeBlock1,' ',0),'-',0).toInt();
  month1  = splitter(splitter(timeBlock1,' ',0),'-',1).toInt();
  day1    = splitter(splitter(timeBlock1,' ',0),'-',2).toInt();
  hour1   = splitter(splitter(timeBlock1,' ',1),'-',0).toInt();
  minute1 = splitter(splitter(timeBlock1,' ',1),'-',1).toInt();
  second1 = splitter(splitter(timeBlock1,' ',1),'-',2).toInt();

  year2   = splitter(splitter(timeBlock2,' ',0),'-',0).toInt();
  month2  = splitter(splitter(timeBlock2,' ',0),'-',1).toInt();
  day2    = splitter(splitter(timeBlock2,' ',0),'-',2).toInt();
  hour2   = splitter(splitter(timeBlock2,' ',1),'-',0).toInt();
  minute2 = splitter(splitter(timeBlock2,' ',1),'-',1).toInt();
  second2 = splitter(splitter(timeBlock2,' ',1),'-',2).toInt();

  if(year1-year2 > 0) return timeBlock1;
  else if(year1-year2 < 0) return timeBlock2;

  if(month1-month2 > 0) return timeBlock1;
  else if(month1-month2 < 0) return timeBlock2;
  
  if(day1-day2 > 0) return timeBlock1;
  else if(month1-month2 < 0) return timeBlock2;
  
  if(hour1-hour2 > 0) return timeBlock1;
  else if(month1-month2 < 0) return timeBlock2;
  
  if(minute1-minute2 > 0) return timeBlock1;
  else if(month1-month2 < 0) return timeBlock2;
  
  if(second1-second2 > 0) return timeBlock1;
  else if(month1-month2 < 0) return timeBlock2;

  return timeBlock1;
}
void changeTime(uint8_t address,String NewTime){
  deviceDatabase[address]["time"] = NewTime.c_str();
  writeMessage2Screen("time changed to :" + NewTime);
}
