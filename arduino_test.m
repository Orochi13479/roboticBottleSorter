clc;
close all;
clear;

ledPin = 'D4';
buttonPin = 'D2';
deltaT_blink = 0.5;

a = arduino;

a.configurePin(buttonPin, 'Pullup');
buttonState = 1;
isBlinking = true;

while isBlinking
    buttonState = a.readDigitalPin(buttonPin);
    if buttonState == 0
        isBlinking = ~isBlinking; % Toggle the blinking state
    end

    if isBlinking
        for k = 1:10
            a.writeDigitalPin(ledPin, 0);
            pause(deltaT_blink/2)

            a.writeDigitalPin(ledPin, 1);
            pause(deltaT_blink/2);

            buttonState = a.readDigitalPin(buttonPin);
            if buttonState == 0
                isBlinking = ~isBlinking; % Toggle the blinking state
                break;
            end
        end
    end
end

