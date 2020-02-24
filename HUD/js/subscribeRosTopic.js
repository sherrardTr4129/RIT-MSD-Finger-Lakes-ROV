var i = 0;
var ros = {};

function updateIMUDataFeild()
{
  var elem = document.getElementById("rovPoseEstimationArea")
  const url = "http://localhost:5000/imu";
  var getObj = new XMLHttpRequest();
  getObj.open("GET", url);
  getObj.responseType = 'text';

  getObj.onload = function()
  {
    if(getObj.readyState === getObj.DONE)
    {
      if(getObj.status === 200)
      {
        var responseText = getObj.response;
        var splitArray = responseText.split("!");
        var timeStamp = parseFloat(splitArray[0])*1000;
        var sensorArray = splitArray[1].split(",");

        var accelX = sensorArray[0].split("(")[1];
        var accelY = sensorArray[1];
        var accelZ = sensorArray[2];

        var magX = sensorArray[3];
        var magY = sensorArray[4];
        var magZ = sensorArray[5];

        var gyroX = sensorArray[6];
        var gyroY = sensorArray[7];
        var gyroZ = sensorArray[8].split(")")[0];

        var date = new Date(timeStamp);
        var hours = date.getHours();
        var minutes = "0" + date.getMinutes();
        var seconds = "0" + date.getSeconds();
        var formattedTime = hours + ":" + minutes.substr(-2) + ":" + seconds.substr(-2);

        var accelString = "X_accel: " + parseFloat(accelX).toFixed(3) + " m/s^2, " + "Y_accel: " + 
            parseFloat(accelY).toFixed(3) + " m/s^2, " + "Z_accel:" + parseFloat(accelZ).toFixed(3) + " m/s^2";

        var magString = "X_mag: " + parseFloat(magX).toFixed(3) + " G, " + "Y_mag: " + parseFloat(magY).toFixed(3) 
            + " G, " + "Z_mag: " + parseFloat(magZ).toFixed(3) + " G";

        var gyroString = "Gyro_X: " + parseFloat(gyroX).toFixed(3) + " deg/sec, " + "Gyro_Y: " + parseFloat(gyroY).toFixed(3) 
            + " deg/sec, " + "Gyro_Z: " + parseFloat(gyroZ).toFixed(3) + " deg/sec"

        var messageToPrint = "Message TimeStamp: " + formattedTime + "\n" + "\n" + accelString + "\n" + "\n" + magString + "\n" + "\n" + gyroString;
        elem.value = messageToPrint;
      }
    }
  }
  getObj.send(" ");
}

var myVar = setInterval(updateIMUDataFeild, 1000);