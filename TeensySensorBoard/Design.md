# Analog Block
There are 8 analog connectors labeled ***ALG0*** through ***ALG7***.
Each connector in the picture has pin 1 at the top and pin 3 at the bottom. Each pin's function is:
1. 3.3 volts. Can be used as a voltage reference or to power the sensor.
2. Ground.
3. Analog signal. Must be in the voltage range of 0 to 3.3.


![Analog block of connectors](AnalogBlock.png)

To use the analog block, you would have code similar to this:

```code
  // Set the analog resolution, in bits for all analog pins.
  analogReadResolution(10);

  // Make sure the pin is not in output mode.
  // In this example, pin 24 corresponds to the port 
  // for the ALG0 connector.
  pinMode(24, INPUT);

  // Read the analog voltage.
  int16_t value = analogRead(24);
```

For a practical example, consider the TMP36 temperature
sensor from [AdaFruit](https://learn.adafruit.com/tmp36-temperature-sensor/overview). Their discussion gives:

> Temp in °C = [(Vout in mV) - 500] / 10
>
> So for example, if the voltage out is 1V that means that the temperature is ((1000 mV - 500) / 10) = 50 °C

Here is code that would read the sensor from the ***ALG0***
port and convert the result to a floating point value representing
tenths of a degree Centigrade.

```code
  // Get a reading that will read 0 to 3.3 volts spread
  // over 1024 buckets (10-bits).
  int raw = analogRead(24);

  // Scale and remove the offset voltage.
  float degreesC = ((raw * 3300 / 1024.0) - 0.5) / 10.0;
```

The pin number to be used for the [pinMode] and [analogRead] values are:

<div style="border:1px solid black; padding: 8px;">
<table>
  <tr>
    <th>Connector</th>
    <th>Pin Number</th>
  </tr>
  <tr>
    <td>ALG0</td>
    <td>24</td>
  </tr>
  <tr>
    <td>ALG1</td>
    <td>25</td>
  </tr>
  <tr>
    <td>ALG2</td>
    <td>26</td>
  </tr>
  <tr>
    <td>ALG3</td>
    <td>27</td>
  </tr>
  <tr>
    <td>ALG4</td>
    <td>38</td>
  </tr>
  <tr>
    <td>ALG5</td>
    <td>39</td>
  </tr>
  <tr>
    <td>ALG6</td>
    <td>40</td>
  </tr>
  <tr>
    <td>ALG7</td>
    <td>41</td>
  </tr>
</table>
</div>

<style>
table {
  border-collapse: collapse;
  width: 100%;
}
th, td {
  border: 1px solid black;
  text-align: left;
  padding: 8px;
}
th {
  background-color: #f2f2f2;
}
</style>

# Relay Block

There are 4 relay connectors labeled ***RLY0*** through ***RLY3***.
The intent is that each connector acts as a switch. The output will
either be driven to ground or allowed to float, which means the switch
signal should be connected to a pull up resistor which then is connected
to the desired 'on' voltage for the switch. That maximum voltage
that can be switched is 5 volts.

Each connector in the picture has pin 1 at the top and pin 3 at the bottom. Each pin's function is:
1. The switched output. The output should include a pull up resistor to
the desired voltage, typically 3.3 volts or 5 volts. Don't exceed 5 volts.
In my original design, the pin was used to drive a solid state relay.
2. Ground.
3. A 5 volt reference. Can be used to power the switched device and/or
be used for that pull up resistor needed for pin 1.

![Relay block of connectors](RelayBlock.png)

To use the relay block, you would have code similar to this:

```code
  // Set the pin in output mode.
  // Here, 0 corresponds to the pin for RLY0.
  pinMode(0, OUTPUT);

  // Turn the switch off.
  digitalWrite(0, LOW);

  // Turn the switch on.
  digitalWrite(0, HIGH);
```

The pin number to be used for the [pinMode] and [digitalWrite] values are:

<div style="border:1px solid black; padding: 8px;">
<table>
  <tr>
    <th>Connector</th>
    <th>Pin Number</th>
  </tr>
  <tr>
    <td>RLY0</td>
    <td>0</td>
  </tr>
  <tr>
    <td>RLY1</td>
    <td>1</td>
  </tr>
  <tr>
    <td>RLY2</td>
    <td>2</td>
  </tr>
  <tr>
    <td>RLY3</td>
    <td>3</td>
  </tr>
</table>
</div>