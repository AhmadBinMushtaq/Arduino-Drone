#include <Arduino.h>
#include "Constants.h"
#include "Functions.h"
#include <TaskManagerIO.h>

void initializeOutputSignals() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);
}

void syncOutputSignals() {
  taskManager.runLoop();
}

void quadcopter_initialization_completed() {
  blink_led();
  taskManager.execute([] {
    tone(BUZZER_PIN, 2000);
    taskManager.scheduleOnce(1000, [] {
      noTone(BUZZER_PIN);
    });
  });
}

bool ledIsOn = false;
int ledBlinkTaskId;

void blink_led() {
  ledBlinkTaskId = taskManager.scheduleFixedRate(500, [] {
    if (ledIsOn) {
      digitalWrite(LED_PIN, LOW);
    } else{
      digitalWrite(LED_PIN, HIGH);
    }
    ledIsOn = !ledIsOn;
  });
}

void quadcopter_armed() {
  blink_led();
  taskManager.execute([] {
    tone(BUZZER_PIN, 200);
    taskManager.scheduleOnce(500, [] {
      tone(BUZZER_PIN, 1000);
      taskManager.scheduleOnce(500, [] {
        tone(BUZZER_PIN, 300);
        taskManager.scheduleOnce(500, [] {
          noTone(BUZZER_PIN);
        });
      });
    });
  });
}

void quadcopter_disarmed() {
  taskManager.cancelTask(ledBlinkTaskId);
  
  taskManager.execute([] {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 3000);
    taskManager.scheduleOnce(500, [] {
      tone(BUZZER_PIN, 1000);
      taskManager.scheduleOnce(500, [] {
        tone(BUZZER_PIN, 200);
        taskManager.scheduleOnce(500, [] {
          noTone(BUZZER_PIN);
        });
      });
    });
  });
}
