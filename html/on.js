String.format = function(){var s = arguments[0];for (var i = 0; i < arguments.length - 1; i++) 
{var reg = new RegExp("\\{" + i + "\\}", "gm");
s = s.replace(reg, arguments[i + 1]);}return s;};
 window.onload=function(e) {myFunction_on();myFunction_on()};
 
function myFunction_on() 
{var t = document.getElementById("temp_on").value;
var x = (t%60);
if ( x < 10 ){ x = "0" + x; }var y = (t-x)/60;
if ( y < 10 ){ y = "0" + y; }
document.getElementById("sh_on").innerHTML = "" + y + ":" + x;}
