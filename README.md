# si5351-stm32
Port of Adafruit's si5351 driver to STM32 HAL<br>
Change the #include on line 49 of si5351.c as needed. <br>
Change that to your board f1, f3, f7 what have you. <br>
<br>
Usage:<br>
  si5351_Init();<br>
  // Set clock 0 to 16MHz<br>
   // 25mhz crystal osc * 32 == 800MHz<br>
   // 800MHz / 50 = 16Mhz<br>
   //<br>
  si5351_setupPLLInt(SI5351_PLL_A, 32);<br>
  si5351_setupMultisynthInt(0, SI5351_PLL_A, 50);<br>
  si5351_setupRdiv(0, SI5351_R_DIV_1);<br>
<br>
  //Set clock 2 to 32.768KHz<br>
   // 25mhz crystal osc * (28 + (7012/390625)) = 700.448768<br>
   // 700.448768 / (1336 + (0 / 1)) = .524288<br>
   // .524288 / 16 = .032768 = 32.768KHz<br>
   <br>
  si5351_setupPLL(SI5351_PLL_B, 28, 7012, 390625);<br>
  si5351_setupMultisynth(2, SI5351_PLL_B, 1336, 0, 1);<br>
  si5351_setupRdiv(2, SI5351_R_DIV_16);<br>
<br>
  si5351_enableOutputs(0xFF);<br>
  <br>
