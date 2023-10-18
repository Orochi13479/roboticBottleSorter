clc;
close all
clear;

ledPin = 'D4';
deltaT_blink = 0.5;

port = 'COM3';
board = 'Due';

a = arduino;

for k =1:10
    a.writeDigitalPin(ledPin, 0);
    pause(deltaT_blink/2)

    a.writeDigitalPin(ledPin, 1);
    pause(deltaT_blink/2);
end

