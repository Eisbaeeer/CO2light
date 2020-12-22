
const char _PAGE_HEAD[] PROGMEM = R"=====(
<HTML>
	<HEAD>
			<TITLE>CO2Light</TITLE>
            <style>

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

	</HEAD>


<BODY>
)=====";
const char _PAGE_START[] PROGMEM  = "<h1>CO2light</h1><form method=get action=/config>";

const char _PAGE_CONFIG_TEMPERATURE[] PROGMEM  = "<label>Temperature: %TEMPERATURE%°C</label><br>";
const char _PAGE_CONFIG_HUMIDITY[] PROGMEM  = "<label>Humidity: %HUMIDITY% %</label><br>";
const char _PAGE_CONFIG_CO2[] PROGMEM  = "<label>CO2: %CO2% ppm</label><br>";
const char _PAGE_CONFIG_INTENSITY[] PROGMEM  = "<label>Helligkeit</label> <input type=range min=0 max=255 name=intensity value=%INTENSITY% ><br> </br>" ;

const char _PAGE_ACTIONS[] PROGMEM = "<label>Erweiterte Funktionen</label>    <a href='/reset'>Reboot</a>   <a href='/firmware'>Firmware upload</a>";
const char _PAGE_SOURCE[] PROGMEM = "<p><label>Infos</label>    <a href='http://www.kidbuild.de'>KidBuild</a> <a href='http://www.kidbuild.de'>%VERSION%</a>";
const char _PAGE_FOOTER[] PROGMEM = "<button type=submit>Konfiguration speichern!</button></form></BODY></HTML>";
