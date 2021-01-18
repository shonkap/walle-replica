// variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
LedControl lc=LedControl(7,6,5,1);

float voltarray[4] = {4.2,4.2,4.2,4.2};
int lowbatcounter = 0;
//all on
byte al[8] = {B11111100,B00001100,B11111100,B11110000,B11110000,B11110000,B11110000,B11110000};
//second bat off
byte bl[8] = {B11111100,B00001100,B00001100,B11110000,B11110000,B11110000,B11110000,B11110000};
//third bat off
byte cl[8] = {B11111100,B00001100,B00001100,B00000000,B11110000,B11110000,B11110000,B11110000};
//fourth bat off
byte dl[8] = {B11111100,B00001100,B00001100,B00000000,B00000000,B11110000,B11110000,B11110000};;
//fifth bat off
byte el[8] = {B11111100,B00001100,B00001100,B00000000,B00000000,B00000000,B11110000,B11110000};
//sixth bat off
byte br[8] = {B11111100,B00001100,B00001100,B00000000,B00000000,B00000000,B00000000,B11110000};
//sixth bat off
byte trf[8] ={B00000000,B00001100,B00001100,B00000000,B00000000,B00000000,B00000000,B00000000};


// Print to Matrix
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
void batvolt(float voltage){
  voltarray[3] = voltarray[2];
  voltarray[2] = voltarray[1];
  voltarray[1] = voltarray[0];
  voltarray[0] = voltage;
  voltage = (voltarray[3]+voltarray[2]+voltarray[1]+voltarray[0])/4;

  //.184 per cell

  //full
  if(voltage >= 4.1){
    for(int i=0;i<8;i++){
      lc.setRow(0,i,al[i]);
    }
    return;
  }
  
  if(voltage >= 3.95){
    for(int i=0;i<8;i++){
      lc.setRow(0,i,bl[i]);
    }
    return;
  }
  
  if(voltage >= 3.8){
    for(int i=0;i<8;i++){
      lc.setRow(0,i,cl[i]);
    }
    return;
  }
  
  if(voltage >= 3.65){
    for(int i=0;i<8;i++){
      lc.setRow(0,i,dl[i]);
    }
    return;
  }

  if(voltage >= 3.5){
    for(int i=0;i<8;i++){
      lc.setRow(0,i,el[i]);
    }
    return;
  }

  if(voltage < 3.5){
    for(int i=0;i<8;i++){
      lc.setRow(0,i,br[i]);
    }
    return;
  }
 
}
