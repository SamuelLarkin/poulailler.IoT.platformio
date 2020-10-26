/* 
 *  [PROGMEM](https://www.arduino.cc/reference/tr/language/variables/utilities/progmem/)
 *  Description
 *  
 *  Store data in flash (program) memory instead of SRAM. There’s a description
 *  of the various types of memory available on an Arduino board.
 *  
 *  The PROGMEM keyword is a variable modifier, it should be used only with the
 *  datatypes defined in pgmspace.h. It tells the compiler "put this information
 *  into flash memory", instead of into SRAM, where it would normally go.
*/
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>&#x1F414; M&eacute;triques du poulailler</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
  <style>
    html {
     font-family: Arial;
     display: inline-block;
     margin: 0px auto;
     text-align: center;
    }
    h2 { font-size: 3.0rem; }
    p { font-size: 3.0rem; }
    .units { font-size: 1.2rem; }
    .dht-labels{
      font-size: 1.5rem;
      vertical-align:middle;
      padding-bottom: 15px;
    }
  </style>
</head>

<body>
  <h2>&#x1F414; <i class="fas fa-egg"></i> M&eacute;triques du poulailler</h2>
  <p>
    <span id="date"></span>
  </p>

  <p>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Temp&eacute;rature</span> 
    <span id="DHT22_temperature">%TEMPERATURE%</span>
    <sup class="units">&deg;C</sup>
  </p>
  <p>
    <i class="fas fa-tint" style="color:#00add6;"></i> 
    <span class="dht-labels">Humidit&eacute;</span>
    <span id="DHT22_humidity">%HUMIDITY%</span>
    <sup class="units">&#37;</sup>
  </p>

  <ul>
    <p>
      <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
      <span class="dht-labels">DS18B20 Temp&eacute;rature</span> 
      <span class="DS18B20_temperature">%DTEMPERATURE%</span>
      <sup class="units">&deg;C</sup>
    </p>
    <p>
      <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
      <span class="dht-labels">DS18B20 Temp&eacute;rature</span> 
      <span class="DS18B20_temperature">%DTEMPERATURE%</span>
      <sup class="units">&deg;C</sup>
    </p>
    <p>
      <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
      <span class="dht-labels">DS18B20 Temp&eacute;rature</span> 
      <span class="DS18B20_temperature">%DTEMPERATURE%</span>
      <sup class="units">&deg;C</sup>
    </p>
    <p>
      <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
      <span class="dht-labels">DS18B20 Temp&eacute;rature</span> 
      <span class="DS18B20_temperature">%DTEMPERATURE%</span>
      <sup class="units">&deg;C</sup>
    </p>
    <p>
      <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
      <span class="dht-labels">DS18B20 Temp&eacute;rature</span> 
      <span class="DS18B20_temperature">%DTEMPERATURE%</span>
      <sup class="units">&deg;C</sup>
    </p>
  </ul>
</body>

<script>
  if (!Date.prototype.toSamString) {
    (function() {

      function pad(number) {
        if (number < 10) {
          return '0' + number;
        }
        return number;
      }

      Date.prototype.toSamString = function() {
        return this.getFullYear()
          + '-' + pad(this.getMonth() + 1)
          + '-' + pad(this.getDate())
          + ' ' + pad(this.getHours())
          + ':' + pad(this.getMinutes())
          + ':' + pad(this.getSeconds());
      };

    })();
  }

  function updateMetriques( ) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        let metriques = JSON.parse(this.responseText);
        let date = new Date(Date.parse(metriques.time));
        document.getElementById("date").innerHTML              = date.toSamString();
        document.getElementById("DHT22_humidity").innerHTML    = metriques.DHT22.humidity;
        document.getElementById("DHT22_temperature").innerHTML = metriques.DHT22.temperature;
        metriques.DS18B20.forEach(function(value, index) {
          document.getElementsByClassName("DS18B20_temperature")[index].innerHTML = value.temperature;
        });
      }
    };
    xhttp.open("GET", "/metriques", true);
    xhttp.send();
  }

  setInterval(updateMetriques, 10000) ;
  updateMetriques();
</script>
</html>)rawliteral";