# _Blinds PWM_  

PWM Blinds is an automatic opener and closer for indoor blinds. It operates on a timer that adjusts the blinds' position depending on the time of day.  
This is a personal project, and the purpose behind it is to explore different development boards such as the ESP32 while combining theoretical knowledge with my interest in automating everyday life.  

## How It's Made  

At the start of the project, a **ledc timer configuration** is created. LEDC is primarily used to control the intensity of LEDs, but such controllers can also be used to generate PWM signals. After that, a **ledc PWM channel** is configured with the correct settings to move the servo back and forth without issues.  

I set up and calculated the correct **duty cycle, duty resolution, steps, cycles, and frequencies** according to the servo's datasheet.

In this project, I use the **Towers pro micro servo 9g SG90** due to it being cheap and flexible.

Inside the `while` loop, the duty cycle increments/decrements in steps depending on the desired angle of the servo. This is done once during init to ensure it is set up correctly.  

Afterwards, a function is called to wait for the appropriate time to shift the blinds. The timer used is a real-time clock that starts at 0 upon program initialization.  

Once the waiting period is over, the loop triggers a **GPIO pin**, which is connected to both an LED and a buzzer, indicating that a blinds shift is about to happen. The servo direction also reverses, so it now operates in the opposite direction.  

## Optimizations  

Since the project is relatively simple and not too large in scope, many optimizations could be made for future development:  

- **Manual control for altering the blinds**: Adding a button near the station would allow users to manually open or close the blinds at any time, rather than relying solely on automation. The best way to implement this would be to add a **manual and automatic mode setting**, so users can choose based on their preference.  
- **NTP for accurate real-time tracking**: By connecting to an **NTP server**, the timing of the servo events would be more precise. This is not currently implemented due to the project's limited scope, as the goal was to keep it small and compact.  

## Lessons Learned  

I was able to dig deeper into **PWM and motor control**. By using the ESP32, I gained hands-on experience with a **top-tier SoC** and programmed it using **ESP-IDF in Visual Studio**.  

I found the ESP32 to be a fascinating piece
  
## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
