const char _PAGE_HEAD[] PROGMEM = R"=====(
<!doctype html>
<html>

<head>
  <title>CO2light by Kidbuild</title>
  <script src = "https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.7.3/Chart.min.js"></script>  
  <style>
  canvas{
    -moz-user-select: none;
    -webkit-user-select: none;
    -ms-user-select: none;
  }

  /* Data Table Styling */
  #dataTable {
    font-family: sans-serif;
    border-collapse: collapse;
    width: 100%;
  }

  #dataTable td, #dataTable th {
    border: 1px solid #ddd;
    padding: 8px;
  }

  #dataTable tr:nth-child(even){background-color: #f2f2f2;}

  #dataTable tr:hover {background-color: #ddd;}

  #dataTable th {
    padding-top: 12px;
    padding-bottom: 12px;
    text-align: left;
    background-color: #4CAF50;
    color: white;
  }

                html {
                font-size: 62.5%;
                box-sizing: border-box;
                }

                html  {
                    padding: 1em;
                }

                *, *::before, *::after {
                margin: 0;
                padding: 0;
                box-sizing: inherit;
                font-family: sans-serif;
                }


                label {
                    display: block;
                    font-size: 18px;
                    border-bottom: 1px solid #eee; 
                    padding: 5px 0;
                    margin: 10px 0 0 0; 
                }

                label.inline {

                    display: block;
                    font-size: 13px;
                    border-bottom: 0 none;
                    padding: 2px 0;
                    margin: 0;
                }

                button, a { 
                    display: inline-block; 
                    padding: 5px 12px 3px 12px; 
                    margin: 2px; 
                    background: #eee;
                    color: #000;
                    border: 0;
                    text-decoration: none;
                }
                
                h1 { font-size: 24px; margin: 5px 0;}
  </style>
   <meta name = "viewport" content = "width = device-width">
   <meta http-equiv="Content-Type" content="text/html; charset=utf-8"/>

</head>

<body>
    <div style="text-align:center;"><b>www.kidbuild.de</b></div>
    <div class="chart-container" position: relative; height:350px; width:100%">
        <canvas id="Chart" width="400" height="400"></canvas>
    </div>
<div>
  <table id="dataTable">
    <tr><th>Time</th><th>CO2 Value</th></tr>
  </table>
</div>
<br>
<br>  

<script>
//Graphs visit: https://www.chartjs.org
var values = [];
var timeStamp = [];
function showGraph()
{
    for (i = 0; i < arguments.length; i++) {
      values.push(arguments[i]);    
    }

    var ctx = document.getElementById("Chart").getContext('2d');
    var Chart2 = new Chart(ctx, {
        type: 'line',
        data: {
            labels: timeStamp,  //Bottom Labeling
            datasets: [{
                label: "CO2 Value",
                fill: false,  //Try with true
                backgroundColor: 'rgba( 243, 156, 18 , 1)', //Dot marker color
                borderColor: 'rgba( 243, 156, 18 , 1)', //Graph Line Color
                data: values,
            }],
        },
        options: {

            maintainAspectRatio: false,
            elements: {
            line: {
                    tension: 0.5 //Smoothening (Curved) of data lines
                }
            },
            scales: {
                    yAxes: [{
                        ticks: {
                            beginAtZero:true
                        }
                    }]
            }
        }
    });

}

//On Page load show graphs
window.onload = function() {
  console.log(new Date().toLocaleTimeString());
};

setInterval(function() {
  // Call a function repetatively with 5 Second interval
  getData();
}, 15000); //15 Seconds update rate
 
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
     //Push the data in array
  var time = new Date().toLocaleTimeString();
  var CO2Value = this.responseText; 
      values.push(CO2Value);
      timeStamp.push(time);
      showGraph();  //Update Graphs
  //Update Data Table
    var table = document.getElementById("dataTable");
    var row = table.insertRow(1); //Add after headings
    var cell1 = row.insertCell(0);
    var cell2 = row.insertCell(1);
    cell1.innerHTML = time;
    cell2.innerHTML = CO2Value;
    }
  };
  xhttp.open("GET", "readValues", true); //Handle readValues server on ESP8266
  xhttp.send();
}
    
</script>
</body>

</html>

)=====";
const char _PAGE_START[] PROGMEM  = "<h1>CO2light by Kidbuild</h1><form method=get action=/config>";

const char _PAGE_CONFIG_TEMPERATURE[] PROGMEM  = "<label>Temperature: %TEMPERATURE%°C</label><br>";
const char _PAGE_CONFIG_HUMIDITY[] PROGMEM  = "<label>Humidity: %HUMIDITY% %</label><br>";
const char _PAGE_CONFIG_CO2[] PROGMEM  = "<label>CO2: %CO2% ppm</label><br>";
const char _PAGE_CONFIG_INTENSITY[] PROGMEM  = "<label>Helligkeit</label> <input type=range min=0 max=255 name=intensity value=%INTENSITY% ><br> </br>" ;

const char _PAGE_ACTIONS[] PROGMEM = "<label>Erweiterte Funktionen</label>    <a href='/reset'>Reboot</a>   <a href='/firmware'>Firmware upload</a>";
const char _PAGE_SOURCE[] PROGMEM = "<p><label>Infos</label>    <a href='http://www.kidbuild.de'>KidBuild</a> <a href='http://www.kidbuild.de'>%VERSION%</a>";
const char _PAGE_FOOTER[] PROGMEM = "<button type=submit>Konfiguration speichern!</button></form></BODY></HTML>";
