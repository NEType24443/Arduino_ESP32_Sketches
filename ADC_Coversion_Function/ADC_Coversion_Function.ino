void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

//*************************************** Analog to Digital Conversion Function ****************************
float A2D(float SenVal)
{
if(SenVal<1) return(0);
else if (SenVal>4094) return(99);
else return(-0.000000000023926 * pow(SenVal,3) + 0.000000094746 * pow(SenVal,2) + 0.00074539 * SenVal + 0.14925);

}
