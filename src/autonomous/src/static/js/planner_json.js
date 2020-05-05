//  This is the main data that will be displayed on the screen
var autonData = null

//  This is the selected used for making changes to the main data
var currentSelectedPath = null;
var currentSelctedPathIndex = null

var currentSelctedGoalIndex = null;

// This updates and saves the page data into the server
function postJSON()
{
  if (autonData != null){
    var xhr = new XMLHttpRequest(); 
    var url = "http://127.0.0.1:5000/planner/api/auton"; 

    // open a connection 
    xhr.open("POST", url, true); 

    // Set the request header i.e. which type of content you are sending 
    xhr.setRequestHeader("Content-Type", "application/json");

    // Converting JSON data to string 
    var data = JSON.stringify(autonData);
    
    // Sending data with the request
    xhr.send(data);
    console.log("Successfully POSTed JSON data")
  }
  else{
    console.log("Did not POST because data is not yet updated from server")
  }
  
}

// This updates the page data from the saves server data
var getAutonJSON = function(url, callback) {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', url, true);
  xhr.responseType = 'json';
  xhr.onload = function() {
    var status = xhr.status;
    if (status === 200) {
      callback(null, xhr.response);
    } else {
      callback(status, xhr.response);
    }
  };
  xhr.send();
};

function getCallback(err, data) {
  if (err !== null) {
    console.log(err);
  } else {
    console.log(data);
    console.log("Successfully loaded in the data from server")
    autonData = data
  }
}