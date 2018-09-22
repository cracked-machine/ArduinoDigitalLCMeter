// 3rd party libs
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 9
Adafruit_SSD1306 display(OLED_RESET);

void lcd_print(String msg) 
{
  display.print(msg);

}

void lcd_printLn(String msg) 
{
  
  display.println(msg);

}

void lcd_reset() 
{
  display.clearDisplay();
  display.setCursor(0,0);
 
}

void lcd_setText(int pColor, int pSize) 
{
  display.setTextSize(pSize);
  display.setTextColor(pColor);
}

void lcd_update() 
{
    display.display();
}

