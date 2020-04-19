//  This is the main data that will be displayed on the screen
var autonData = null

//  This is the selected used for making changes to the main data
var currentSelectedPath = null;
var currentSelctedPathIndex = null

var currentSelctedGoal = null;
var currentSelctedGoalIndex = null;

// Robot pose data
var currentRobotPose = null;
var startRobotPose = null;

// For controlling the slideouts
function left_show()
{
    document.getElementById('left_sidebar').classList.toggle('active')
}

function right_show()
{
    document.getElementById('right_sidebar').classList.toggle('active')
}

// For controlling the tab content
function openLeft(evt, tabName)
{
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("left_tabcontent");
    for (i = 0; i < tabcontent.length; i++) {
      tabcontent[i].style.display = "none";
    }
    tablinks = document.getElementsByClassName("left_tablinks");
    for (i = 0; i < tablinks.length; i++) {
      tablinks[i].className = tablinks[i].className.replace(" active", "");
    }
    document.getElementById(tabName).style.display = "block";
    evt.currentTarget.className += " active";
}

function openRight(evt, tabName)
{
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("right_tabcontent");
    for (i = 0; i < tabcontent.length; i++) {
      tabcontent[i].style.display = "none";
    }
    tablinks = document.getElementsByClassName("right_tablinks");
    for (i = 0; i < tablinks.length; i++) {
      tablinks[i].className = tablinks[i].className.replace(" active", "");
    }

    tabcontent = document.getElementsByClassName("GoalContent");
    for (i = 0; i < tabcontent.length; i++) {
      tabcontent[i].style.display = "none";
    }
    
    document.getElementById(tabName).style.display = "block";
    evt.currentTarget.className += " active";
}

// This updates and saves the page data into the server
function postJSON()
{
  if (autonData != null){
    var xhr = new XMLHttpRequest(); 
    var url = "http://localhost:5000/planner/api/auton"; 

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

// Robot Pose
function postRobotStartJSON()
{
  var xhr = new XMLHttpRequest(); 
  var url = "http://localhost:5000/planner/api/robot_pose"; 

  // open a connection 
  xhr.open("POST", url, true); 

  // Set the request header i.e. which type of content you are sending 
  xhr.setRequestHeader("Content-Type", "application/json");

  // Converting JSON data to string 
  var data = JSON.stringify({});
  
  // Sending data with the request
  xhr.send(data);
  console.log("Successfully POSTed JSON data")
  
}

function getPoseCallback(err, data) {
  if (err !== null) {
    console.log(err);
  } else {
    console.log(data);
    console.log("Successfully loaded in robot Pose Data")
    currentRobotPose = data
  }
}

// PATHS

function addPath(){

  var elements = document.getElementsByClassName("InputLeft");
    while(elements.length > 0){
        elements[0].parentNode.removeChild(elements[0]);
    }

  var leftSlideBar = document.getElementById("left_sidebar");

  var myInput = document.createElement('INPUT');
  myInput.id = "myPathInput"
  myInput.setAttribute("type", "text");
  myInput.className = "InputLeft"

  var myButton = document.createElement('BUTTON')
  myButton.id = "myPathButton"
  myButton.setAttribute("type", "button");
  myButton.textContent = "Create"
  myButton.className = "InputLeft"

  leftSlideBar.appendChild(myInput)
  leftSlideBar.appendChild(myButton)

  myButton.onclick = function() { 
    addPathInput(); 
}

}

function addPathInput(){
  var inputVal = document.getElementById("myPathInput").value;
  console.log("Created a new path")
  autonData.paths.push({"id":autonData.paths.length, "goals":[] , "name":inputVal})
  console.log(autonData)
  postJSON();
  displayPathTable();
}

function deletePath(id){
  autonData.paths.splice(id.slice(10,id.length),1);
  var index = id.slice(10,id.length);
  console.log("Deleted Path Index : " + index)
  while (index < autonData.paths.length){
    autonData.paths[index].id -= 1;
    // console.log(autonData.paths[index])
    index++;
  }
  currentSelctedPathIndex = null
  currentSelectedPath = null
  displayPathTable()
  postJSON()

}

// This reads from the auton data and displays all the data dynamicly on the screen
function displayPathTable(){
  var myTableDiv = document.getElementById("left_Path_Table");
  var elements = document.getElementsByClassName("PathOption");
    while(elements.length > 0){
        elements[0].parentNode.removeChild(elements[0]);
    }
  
  var elements = document.getElementsByClassName("PathTrash");
    while(elements.length > 0){
        elements[0].parentNode.removeChild(elements[0]);
    }

  var elements = document.getElementsByClassName("InputLeft");
    while(elements.length > 0){
        elements[0].parentNode.removeChild(elements[0]);
    }

  if (autonData.paths.length == 0){
    // Display no auton paths
  }
  else{

    for (var x in autonData.paths){
      var myDiv = document.createElement('DIV');
      myDiv.className = "PathOption"
      myDiv.id = "Path " + autonData.paths[x].id;

      var myPathTitle = document.createElement('H2')
      myPathTitle.textContent = autonData.paths[x].name;
      myDiv.insertBefore(myPathTitle, myDiv.firstChild);

      var myPathNumGoals = document.createElement('H6');
      myPathNumGoals.textContent = "Num Goals: " + autonData.paths[x].goals.length
      myDiv.insertBefore(myPathNumGoals, myDiv.firstChild);

      var myPathTrash = document.createElement('H4');
      var trash = '\u2716'
      myPathTrash.textContent = trash
      myPathTrash.id = "PathTrash " + autonData.paths[x].id
      myPathTrash.className = "PathTrash"

      myPathTrash.onclick = function() { 
        deletePath(this.id); 
    }

      myTableDiv.insertBefore(myPathTrash, myTableDiv.firstChild);

      myDiv.onclick = function() { 
        selectedPath(this.id); 
    }

      myTableDiv.insertBefore(myDiv, myTableDiv.firstChild);
    }

    if (currentSelectedPath != null){
      document.getElementById("Path " + currentSelctedPathIndex).classList.toggle('active')
    }
    
  }
  
}

// Selects the given Path and adds to data
function selectedPath(id){
  // To reset the color on the prev selected
  if (currentSelectedPath != null){
    document.getElementById("Path " + currentSelctedPathIndex).classList.toggle('active')
  }

  if (currentSelctedPathIndex == id.slice(5,id.length)){
    currentSelectedPath = null
    currentSelctedPathIndex = null
    console.log(currentSelectedPath)
  }
  else{
    document.getElementById(id).classList.toggle('active')
    currentSelectedPath = autonData.paths[id.slice(5,id.length)]
    currentSelctedPathIndex = id.slice(5,id.length)
    console.log(currentSelectedPath)
  }
  displayGoalDropDown()
  
}

// GOALS

function addGoal(){
  if (currentSelectedPath != null){
    var goalsData = autonData.paths[currentSelctedPathIndex].goals
    goalsData.push({"id":goalsData.length, "position_x":null, "position_y":null, "th": null, "constants_kP":1.0, "constants_kA":6.0, "constants_kB":-0.8, "max_linear_speed":1.1, "min_linear_speed":0.1, "max_linear_acceleration":1E9, "linear_tolerance_outer":0.3, "linear_tolerance_inner":0.1, "max_angular_speed":2.0, "min_angular_speed":1.0, "max_angular_acceleration":1E9, "angular_tolerance_outer":0.2, "angular_tolerance_inner":0.1, "ignore_angular_tolerance":false, "forward_movement_only":false})
    console.log("Added Goal to Path")
    console.log(autonData.paths[currentSelctedPathIndex].goals)
    
    // Updates view
    displayPathTable()
    displayGoalDropDown()
    postJSON()
  }
}

function deleteGoal(id){
  autonData.paths[currentSelctedPathIndex].goals.splice(id.slice(10,id.length),1);
  var index = id.slice(10,id.length);
  console.log("Deleted Path Index : " + index)
  while (index < autonData.paths[currentSelctedPathIndex].goals.length){
    autonData.paths[currentSelctedPathIndex].goals[index].id -= 1;
    // console.log(autonData.paths[index])
    index++;
  }
  displayGoalDropDown()
  displayPathTable()
  postJSON()
}

function displayGoalDropDown(){
  var myContent = document.getElementById("dropdown-content");

  var elements = document.getElementsByClassName("GoalOption");
    while(elements.length > 0){
        elements[0].parentNode.removeChild(elements[0]);
    }
    console.log(myContent)

    if (currentSelectedPath != null){

      for (var x in autonData.paths[currentSelctedPathIndex].goals){

        var myGoal = document.createElement('BUTTON');
        myGoal.className = "GoalOption";
        myGoal.id = "Goal " + autonData.paths[currentSelctedPathIndex].goals[x].id;
        var value = autonData.paths[currentSelctedPathIndex].goals[x].id + 1;
        myGoal.textContent = "Goal " + value;
        myGoal.onclick = function() { 
          openRight(event, this.id + "Content");
          document.getElementById("dropdown-content").style.display = "none";  
          document.getElementById(this.id).style.background = ''
      }
        myContent.insertBefore(myGoal, myContent.firstChild);

      }

      displayGoalEditor()
  }

}

function displayGoalEditor(){

  var myRightSideBar = document.getElementById("right_sidebar");

  var elements = document.getElementsByClassName("GoalContent");
    while(elements.length > 0){
        elements[0].parentNode.removeChild(elements[0]);
    }
  

    if (currentSelectedPath != null){

      for (var x in autonData.paths[currentSelctedPathIndex].goals){

        var myDiv = document.createElement('DIV');
        myDiv.className = "GoalContent";
        myDiv.id = "Goal " + autonData.paths[currentSelctedPathIndex].goals[x].id + "Content";
        var value = autonData.paths[currentSelctedPathIndex].goals[x].id + 1;
        var myH1 = document.createElement('H1');
        myH1.textContent = "Goal " + value;

        var myDivData = document.createElement('DIV');
        myDivData.className = "GoalContentData";
        myDivData.id = "GoalContentData" + autonData.paths[currentSelctedPathIndex].goals[x].id;

        var myGoalTrash = document.createElement('H4');
        var trash = '\u2716'
        myGoalTrash.textContent = trash
        myGoalTrash.id = "GoalTrash " + autonData.paths[currentSelctedPathIndex].goals[x].id
        myGoalTrash.className = "GoalTrash"

        myGoalTrash.onclick = function() { 
          deleteGoal(this.id); 
        }

        myDiv.insertBefore(myGoalTrash, myDiv.firstChild);

        myDiv.insertBefore(myDivData, myDiv.firstChild);
        myDiv.insertBefore(myH1, myDiv.firstChild);
        myDiv.insertBefore(myGoalTrash, myDiv.firstChild);

        myRightSideBar.appendChild(myDiv);

        displayGoalPose(autonData.paths[currentSelctedPathIndex].goals[x].id);
        displayGoalConstants(autonData.paths[currentSelctedPathIndex].goals[x].id)
        displayGoalLinear(autonData.paths[currentSelctedPathIndex].goals[x].id);
        displayGoalAngular(autonData.paths[currentSelctedPathIndex].goals[x].id);

      }
  }
}

function onHover(){
  var myContent = document.getElementById("dropdown-content");
  document.getElementById("dropdown-content").style.display = "";
  var elements = document.getElementsByClassName("GoalOption");
    while(elements.length > 0){
        elements[0].parentNode.removeChild(elements[0]);
    }
    console.log(myContent)

    if (currentSelectedPath != null){

      for (var x in autonData.paths[currentSelctedPathIndex].goals){

        var myGoal = document.createElement('BUTTON');
        myGoal.className = "GoalOption";
        myGoal.id = "Goal " + autonData.paths[currentSelctedPathIndex].goals[x].id;
        var value = autonData.paths[currentSelctedPathIndex].goals[x].id + 1;
        myGoal.textContent = "Goal " + value;
        myGoal.onclick = function() { 
          openRight(event, this.id + "Content");
          document.getElementById("dropdown-content").style.display = "none";  
          document.getElementById(this.id).style.background = ''
      }
        myContent.insertBefore(myGoal, myContent.firstChild);

      }
  }
}

function displayGoalPose(x){
  var myGoalContent = document.getElementById("GoalContentData" + x);

  var myDropDown = document.createElement('H5');
  myDropDown.className = "GoalDataDropDown"
  myDropDown.textContent = "Pose Data"
  myDropDown.id = "Pose"
  myDropDown.onclick = function() { 
    if (document.getElementById("PoseGoalDataContent" + x).style.display == "none"){
      document.getElementById("PoseGoalDataContent" + x).style.display = "block";
    }
    else{
      document.getElementById("PoseGoalDataContent" + x).style.display = "none";
    }
  }

  var myDiv = document.createElement('DIV');
  myDiv.id = "PoseGoalDataContent" + x
  myDiv.className = "PoseGoalDataContent"

  // Content
  // 1
  var myXDiv = document.createElement('DIV');
  myXDiv.id = "PoseGoalDataXDiv" + x
  myXDiv.className = "PoseGoalDataDiv"

  var myX = document.createElement('H5');
  myX.id = "PoseGoalDataX" + x
  myX.textContent = "Pose X"
  myX.className = "PoseGoalDataName"

  var myXInput = document.createElement('INPUT');
  myXInput.id = "PoseGoalDataXInput" + x
  myXInput.setAttribute("type", "text");
  myXInput.className = "PoseGoalDataXInput"
  myXInput.onblur = function() {GoalPoseInput(x);}
  myXInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalPoseInput(x);
    }
  }

  myXDiv.appendChild(myX);
  myXDiv.appendChild(myXInput);

  // 2
  var myYDiv = document.createElement('DIV');
  myYDiv.id = "PoseGoalDataYDiv" + x
  myYDiv.className = "PoseGoalDataDiv"

  var myY = document.createElement('H5');
  myY.id = "PoseGoalDataY" + x
  myY.textContent = "Pose Y"
  myY.className = "PoseGoalDataName"

  var myYInput = document.createElement('INPUT');
  myYInput.id = "PoseGoalDataYInput" + x
  myYInput.setAttribute("type", "text");
  myYInput.className = "PoseGoalDataYInput"
  myYInput.onblur = function() {GoalPoseInput(x);}
  myYInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalPoseInput(x);
    }
  }

  myYDiv.appendChild(myY);
  myYDiv.appendChild(myYInput);

  // 3
  var myThDiv = document.createElement('DIV');
  myThDiv.id = "PoseGoalDataThDiv" + x
  myThDiv.className = "PoseGoalDataDiv"

  var myTh = document.createElement('H5');
  myTh.id = "PoseGoalDataTh" + x
  myTh.textContent = "Pose Th"
  myTh.className = "PoseGoalDataName"

  var myThInput = document.createElement('INPUT');
  myThInput.id = "PoseGoalDataThInput" + x
  myThInput.setAttribute("type", "text");
  myThInput.className = "PoseGoalDataThInput"
  myThInput.onblur = function() {GoalPoseInput(x);}
  myThInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalPoseInput(x);
    }
  }

  myThDiv.appendChild(myTh);
  myThDiv.appendChild(myThInput);

  myDiv.appendChild(myXDiv);
  myDiv.appendChild(myYDiv);
  myDiv.appendChild(myThDiv);


  myGoalContent.appendChild(myDropDown)
  myGoalContent.appendChild(myDiv)

  // Set default content for input
  if (autonData.paths[currentSelctedPathIndex].goals[x].position_x == null){
    document.getElementById("PoseGoalDataXInput" + x).placeholder = "Select X Pose";
  }
  else{
    document.getElementById("PoseGoalDataXInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].position_x;
  }

  if (autonData.paths[currentSelctedPathIndex].goals[x].position_y == null){
    document.getElementById("PoseGoalDataYInput" + x).placeholder = "Select Y Pose";
  }
  else{
    document.getElementById("PoseGoalDataYInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].position_y;
  }

  if (autonData.paths[currentSelctedPathIndex].goals[x].t == null){
    document.getElementById("PoseGoalDataThInput" + x).placeholder = "Select Th Pose";
  }
  else{
    document.getElementById("PoseGoalDataThInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].th;
  }
  
}

function GoalPoseInput(x){
  console.log("Pose is working")

  var inputValX = parseFloat(document.getElementById("PoseGoalDataXInput" + x).value);
  if (inputValX == ""){
    inputValX = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].position_x = inputValX

  var inputValY = parseFloat(document.getElementById("PoseGoalDataYInput" + x).value);
  if (inputValY == ""){
    inputValY = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].position_y = inputValY

  var inputValTh = parseFloat(document.getElementById("PoseGoalDataThInput" + x).value);
  if (inputValTh == ""){
    inputValTh = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].th = inputValTh

  postJSON();
}

function displayGoalConstants(x){
  var myGoalContent = document.getElementById("GoalContentData" + x);

  var myDropDown = document.createElement('H5');
  myDropDown.className = "GoalDataDropDown"
  myDropDown.textContent = "Constants Data"
  myDropDown.id = "Constants"
  myDropDown.onclick = function() { 
    if (document.getElementById("ConstantsGoalDataContent" + x).style.display == "none"){
      document.getElementById("ConstantsGoalDataContent" + x).style.display = "block";
    }
    else{
      document.getElementById("ConstantsGoalDataContent" + x).style.display = "none";
    }
  }

  var myDiv = document.createElement('DIV');
  myDiv.id = "ConstantsGoalDataContent" + x
  myDiv.className = "ConstantsGoalDataContent"

  // Content
  // 1
  var myKpDiv = document.createElement('DIV');
  myKpDiv.id = "ConstantsGoalDataKpDiv" + x
  myKpDiv.className = "ConstantsGoalDataDiv"

  var myKp = document.createElement('H5');
  myKp.id = "ConstantsGoalDataKp" + x
  myKp.textContent = "Constant kP"
  myKp.className = "ConstantsGoalDataName"

  var myKpInput = document.createElement('INPUT');
  myKpInput.id = "ConstantsGoalDataKpInput" + x
  myKpInput.setAttribute("type", "text");
  myKpInput.className = "ConstantsGoalDataKpInput"
  myKpInput.onblur = function() {GoalConstantsInput(x);}
  myKpInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalConstantsInput(x);
    }
  }

  myKpDiv.appendChild(myKp);
  myKpDiv.appendChild(myKpInput);

  // 2
  var myKaDiv = document.createElement('DIV');
  myKaDiv.id = "ConstantsGoalDataKaDiv" + x
  myKaDiv.className = "ConstantsGoalDataDiv"

  var myKa = document.createElement('H5');
  myKa.id = "ConstantsGoalDataKa" + x
  myKa.textContent = "Constant kA"
  myKa.className = "ConstantsGoalDataName"

  var myKaInput = document.createElement('INPUT');
  myKaInput.id = "ConstantsGoalDataKaInput" + x
  myKaInput.setAttribute("type", "text");
  myKaInput.className = "ConstantsGoalDataKaInput"
  myKaInput.onblur = function() {GoalConstantsInput(x);}
  myKaInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalConstantsInput(x);
    }
  }

  myKaDiv.appendChild(myKa);
  myKaDiv.appendChild(myKaInput);

  // 3
  var myKbDiv = document.createElement('DIV');
  myKbDiv.id = "ConstantsGoalDataKbDiv" + x
  myKbDiv.className = "ConstantsGoalDataDiv"

  var myKb = document.createElement('H5');
  myKb.id = "ConstantsGoalDataKb" + x
  myKb.textContent = "Constant kB"
  myKb.className = "ConstantsGoalDataName"

  var myKbInput = document.createElement('INPUT');
  myKbInput.id = "ConstantsGoalDataKbInput" + x
  myKbInput.setAttribute("type", "text");
  myKbInput.className = "ConstantsGoalDataKbInput"
  myKbInput.onblur = function() {GoalConstantsInput(x);}
  myKbInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalConstantsInput(x);
    }
  }

  myKbDiv.appendChild(myKb);
  myKbDiv.appendChild(myKbInput);

  myDiv.appendChild(myKpDiv);
  myDiv.appendChild(myKaDiv);
  myDiv.appendChild(myKbDiv);


  myGoalContent.appendChild(myDropDown)
  myGoalContent.appendChild(myDiv)

  // Set default content for input
  //1
  if (autonData.paths[currentSelctedPathIndex].goals[x].constants_kP == null){
    document.getElementById("ConstantsGoalDataKpInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("ConstantsGoalDataKpInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].constants_kP;
  }

  //2
  if (autonData.paths[currentSelctedPathIndex].goals[x].constants_kA == null){
    document.getElementById("ConstantsGoalDataKaInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("ConstantsGoalDataKaInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].constants_kA;
  }

  //3
  if (autonData.paths[currentSelctedPathIndex].goals[x].constants_kB == null){
    document.getElementById("ConstantsGoalDataKbInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("ConstantsGoalDataKbInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].constants_kB;
  }

}

function GoalConstantsInput(x){
  console.log("Constants are working")

  var inputValKp = parseFloat(document.getElementById("ConstantsGoalDataKpInput" + x).value);
  if (inputValKp == ""){
    inputValKp = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].constants_kP = inputValKp

  var inputValKa = parseFloat(document.getElementById("ConstantsGoalDataKaInput" + x).value);
  if (inputValKa == ""){
    inputValKa = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].constants_kA = inputValKa

  var inputValKb = parseFloat(document.getElementById("ConstantsGoalDataKbInput" + x).value);
  if (inputValKb == ""){
    inputValKb = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].constants_kB = inputValKb

  postJSON();
}

function displayGoalLinear(x){
  
  var myGoalContent = document.getElementById("GoalContentData" + x);

  var myDropDown = document.createElement('H5');
  myDropDown.className = "GoalDataDropDown"
  myDropDown.textContent = "Linear Data"
  myDropDown.onclick = function() { 
    if (document.getElementById("LinearGoalDataContent" + x).style.display == "none"){
      document.getElementById("LinearGoalDataContent" + x).style.display = "block";
    }
    else{
      document.getElementById("LinearGoalDataContent" + x).style.display = "none";
    }
  }

  var myDiv = document.createElement('DIV');
  myDiv.id = "LinearGoalDataContent" + x
  myDiv.className = "LinearGoalDataContent"

  // Content
  // 1
  var myMaxSpeedDiv = document.createElement('DIV');
  myMaxSpeedDiv.id = "LinearGoalDataMaxSpeedDiv" + x
  myMaxSpeedDiv.className = "LinearGoalDataDiv"

  var myMaxSpeed = document.createElement('H5');
  myMaxSpeed.id = "LinearGoalDataMaxSpeed" + x
  myMaxSpeed.textContent = "Max Speed"
  myMaxSpeed.className = "LinearGoalDataName"

  var myMaxSpeedInput = document.createElement('INPUT');
  myMaxSpeedInput.id = "LinearGoalDataMaxSpeedInput" + x
  myMaxSpeedInput.setAttribute("type", "text");
  myMaxSpeedInput.className = "LinearGoalDataMaxSpeedInput"
  myMaxSpeedInput.onblur = function() {GoalLinearInput(x);}
  myMaxSpeedInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalLinearInput(x);
    }
  }

  myMaxSpeedDiv.appendChild(myMaxSpeed);
  myMaxSpeedDiv.appendChild(myMaxSpeedInput);

  // 2
  var myMinSpeedDiv = document.createElement('DIV');
  myMinSpeedDiv.id = "LinearGoalDataMinSpeedDiv" + x
  myMinSpeedDiv.className = "LinearGoalDataDiv"

  var myMinSpeed = document.createElement('H5');
  myMinSpeed.id = "LinearGoalDataMinSpeed" + x
  myMinSpeed.textContent = "Min Speed"
  myMinSpeed.className = "LinearGoalDataName"

  var myMinSpeedInput = document.createElement('INPUT');
  myMinSpeedInput.id = "LinearGoalDataMinSpeedInput" + x
  myMinSpeedInput.setAttribute("type", "text");
  myMinSpeedInput.className = "LinearGoalDataMinSpeedInput"
  myMinSpeedInput.onblur = function() {GoalLinearInput(x);}
  myMinSpeedInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalLinearInput(x);
    }
  }

  myMinSpeedDiv.appendChild(myMinSpeed);
  myMinSpeedDiv.appendChild(myMinSpeedInput);

  // 3
  var myMaxAccelDiv = document.createElement('DIV');
  myMaxAccelDiv.id = "LinearGoalDataMaxAccelDiv" + x
  myMaxAccelDiv.className = "LinearGoalDataDiv"

  var myMaxAccel = document.createElement('H5');
  myMaxAccel.id = "LinearGoalDataMaxAccel" + x
  myMaxAccel.textContent = "Max Accel"
  myMaxAccel.className = "LinearGoalDataName"

  var myMaxAccelInput = document.createElement('INPUT');
  myMaxAccelInput.id = "LinearGoalDataMaxAccelInput" + x
  myMaxAccelInput.setAttribute("type", "text");
  myMaxAccelInput.className = "LinearGoalDataMaxAccelInput"
  myMaxAccelInput.onblur = function() {GoalLinearInput(x);}
  myMaxAccelInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalLinearInput(x);
    }
  }

  myMaxAccelDiv.appendChild(myMaxAccel);
  myMaxAccelDiv.appendChild(myMaxAccelInput);

  // 4
  var myTolOuterDiv = document.createElement('DIV');
  myTolOuterDiv.id = "LinearGoalDataTolOuterDiv" + x
  myTolOuterDiv.className = "LinearGoalDataDiv"

  var myTolOuter = document.createElement('H5');
  myTolOuter.id = "LinearGoalDataTolOuter" + x
  myTolOuter.textContent = "Tol Outer"
  myTolOuter.className = "LinearGoalDataName"

  var myTolOuterInput = document.createElement('INPUT');
  myTolOuterInput.id = "LinearGoalDataTolOuterInput" + x
  myTolOuterInput.setAttribute("type", "text");
  myTolOuterInput.className = "LinearGoalDataTolOuterInput"
  myTolOuterInput.onblur = function() {GoalLinearInput(x);}
  myTolOuterInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalLinearInput(x);
    }
  }

  myTolOuterDiv.appendChild(myTolOuter);
  myTolOuterDiv.appendChild(myTolOuterInput);

  // 5
  var myTolInnerDiv = document.createElement('DIV');
  myTolInnerDiv.id = "LinearGoalDataTolInnerDiv" + x
  myTolInnerDiv.className = "LinearGoalDataDiv"

  var myTolInner = document.createElement('H5');
  myTolInner.id = "LinearGoalDataTolInner" + x
  myTolInner.textContent = "Tol Inner"
  myTolInner.className = "LinearGoalDataName"

  var myTolInnerInput = document.createElement('INPUT');
  myTolInnerInput.id = "LinearGoalDataTolInnerInput" + x
  myTolInnerInput.setAttribute("type", "text");
  myTolInnerInput.className = "LinearGoalDataTolInnerInput"
  myTolInnerInput.onblur = function() {GoalLinearInput(x);}
  myTolInnerInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalLinearInput(x);
    }
  }

  myTolInnerDiv.appendChild(myTolInner);
  myTolInnerDiv.appendChild(myTolInnerInput);

  // 6
  var myForMoveOnlyDiv = document.createElement('DIV');
  myForMoveOnlyDiv.id = "LinearGoalDataForMoveOnlyDiv" + x
  myForMoveOnlyDiv.className = "LinearGoalDataDiv"

  var myForMoveOnly = document.createElement('H5');
  myForMoveOnly.id = "LinearGoalDataForMoveOnly" + x
  myForMoveOnly.textContent = "Forward Move Only"
  myForMoveOnly.className = "LinearGoalDataName"

  var myForMoveOnlyInput = document.createElement('INPUT');
  myForMoveOnlyInput.id = "LinearGoalDataForMoveOnlyInput" + x
  myForMoveOnlyInput.setAttribute("type", "checkbox");
  myForMoveOnlyInput.className = "LinearGoalDataForMoveOnlyInput"
  myForMoveOnlyInput.onclick = function(){
    GoalLinearInput(x);
  }

  myForMoveOnlyDiv.appendChild(myForMoveOnly);
  myForMoveOnlyDiv.appendChild(myForMoveOnlyInput);


  myDiv.appendChild(myMaxSpeedDiv); // 1
  myDiv.appendChild(myMinSpeedDiv); // 2
  myDiv.appendChild(myMaxAccelDiv); // 3
  myDiv.appendChild(myTolOuterDiv); // 4
  myDiv.appendChild(myTolInnerDiv); // 5
  myDiv.appendChild(myForMoveOnlyDiv); // 6


  myGoalContent.appendChild(myDropDown)
  myGoalContent.appendChild(myDiv)

  // Set default content for input
  //1
  if (autonData.paths[currentSelctedPathIndex].goals[x].max_linear_speed == null){
    document.getElementById("LinearGoalDataMaxSpeedInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("LinearGoalDataMaxSpeedInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].max_linear_speed;
  }

  //2
  if (autonData.paths[currentSelctedPathIndex].goals[x].min_linear_speed == null){
    document.getElementById("LinearGoalDataMinSpeedInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("LinearGoalDataMinSpeedInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].min_linear_speed;
  }

  //3
  if (autonData.paths[currentSelctedPathIndex].goals[x].max_linear_acceleration == null){
    document.getElementById("LinearGoalDataMaxAccelInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("LinearGoalDataMaxAccelInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].max_linear_acceleration;
  }

  //4
  if (autonData.paths[currentSelctedPathIndex].goals[x].linear_tolerance_outer == null){
    document.getElementById("LinearGoalDataTolOuterInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("LinearGoalDataTolOuterInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].linear_tolerance_outer;
  }

  //5
  if (autonData.paths[currentSelctedPathIndex].goals[x].linear_tolerance_inner == null){
    document.getElementById("LinearGoalDataTolInnerInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("LinearGoalDataTolInnerInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].linear_tolerance_inner;
  }

  //6
  document.getElementById("LinearGoalDataForMoveOnlyInput" + x).checked = autonData.paths[currentSelctedPathIndex].goals[x].forward_movement_only

}

function GoalLinearInput(x){
  console.log("Linear stuff is working")

  var inputValMaxSpeed = parseFloat(document.getElementById("LinearGoalDataMaxSpeedInput" + x).value);
  if (inputValMaxSpeed == ""){
    inputValMaxSpeed = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].max_linear_speed = inputValMaxSpeed

  var inputValMinSpeed = parseFloat(document.getElementById("LinearGoalDataMinSpeedInput" + x).value);
  if (inputValMinSpeed == ""){
    inputValMinSpeed = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].min_linear_speed = inputValMinSpeed

  var inputValMaxAccel = parseFloat(document.getElementById("LinearGoalDataMaxAccelInput" + x).value);
  if (inputValMaxAccel == ""){
    inputValMaxAccel = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].max_linear_acceleration = inputValMaxAccel

  var inputValTolOuter = parseFloat(document.getElementById("LinearGoalDataTolOuterInput" + x).value);
  if (inputValTolOuter == ""){
    inputValTolOuter = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].linear_tolerance_outer = inputValTolOuter

  var inputValTolInner = parseFloat(document.getElementById("LinearGoalDataTolInnerInput" + x).value);
  if (inputValTolInner == ""){
    inputValTolInner = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].linear_tolerance_inner = inputValTolInner

  var inputValForMoveOnly = document.getElementById("LinearGoalDataForMoveOnlyInput" + x).checked;
  console.log(inputValForMoveOnly)
  if (inputValForMoveOnly){
    inputValForMoveOnly = true;
  }
  else{
    inputValForMoveOnly = false;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].forward_movement_only = inputValForMoveOnly

  postJSON();
}

function displayGoalAngular(x){
  var myGoalContent = document.getElementById("GoalContentData" + x);

  var myDropDown = document.createElement('H5');
  myDropDown.className = "GoalDataDropDown"
  myDropDown.textContent = "Angular Data"
  myDropDown.id = "Angle"
  myDropDown.onclick = function() { 
    if (document.getElementById("AngularGoalDataContent" + x).style.display == "none"){
      document.getElementById("AngularGoalDataContent" + x).style.display = "block";
    }
    else{
      document.getElementById("AngularGoalDataContent" + x).style.display = "none";
    }
  }

  var myDiv = document.createElement('DIV');
  myDiv.id = "AngularGoalDataContent" + x
  myDiv.className = "AngularGoalDataContent"

  // Content
  // 1
  var myMaxSpeedDiv = document.createElement('DIV');
  myMaxSpeedDiv.id = "AngularGoalDataMaxSpeedDiv" + x
  myMaxSpeedDiv.className = "AngularGoalDataDiv"

  var myMaxSpeed = document.createElement('H5');
  myMaxSpeed.id = "AngularGoalDataMaxSpeed" + x
  myMaxSpeed.textContent = "Max Speed"
  myMaxSpeed.className = "AngularGoalDataName"

  var myMaxSpeedInput = document.createElement('INPUT');
  myMaxSpeedInput.id = "AngularGoalDataMaxSpeedInput" + x
  myMaxSpeedInput.setAttribute("type", "text");
  myMaxSpeedInput.className = "AngularGoalDataMaxSpeedInput"
  myMaxSpeedInput.onblur = function() {GoalAngularInput(x);}
  myMaxSpeedInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalAngularInput(x);
    }
  }

  myMaxSpeedDiv.appendChild(myMaxSpeed);
  myMaxSpeedDiv.appendChild(myMaxSpeedInput);

  // 2
  var myMinSpeedDiv = document.createElement('DIV');
  myMinSpeedDiv.id = "AngularGoalDataMinSpeedDiv" + x
  myMinSpeedDiv.className = "AngularGoalDataDiv"

  var myMinSpeed = document.createElement('H5');
  myMinSpeed.id = "AngularGoalDataMinSpeed" + x
  myMinSpeed.textContent = "Min Speed"
  myMinSpeed.className = "AngularGoalDataName"

  var myMinSpeedInput = document.createElement('INPUT');
  myMinSpeedInput.id = "AngularGoalDataMinSpeedInput" + x
  myMinSpeedInput.setAttribute("type", "text");
  myMinSpeedInput.className = "AngularGoalDataMinSpeedInput"
  myMinSpeedInput.onblur = function() {GoalAngularInput(x);}
  myMinSpeedInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalAngularInput(x);
    }
  }

  myMinSpeedDiv.appendChild(myMinSpeed);
  myMinSpeedDiv.appendChild(myMinSpeedInput);

  // 3
  var myMaxAccelDiv = document.createElement('DIV');
  myMaxAccelDiv.id = "AngularGoalDataMaxAccelDiv" + x
  myMaxAccelDiv.className = "AngularGoalDataDiv"

  var myMaxAccel = document.createElement('H5');
  myMaxAccel.id = "AngularGoalDataMaxAccel" + x
  myMaxAccel.textContent = "Max Accel"
  myMaxAccel.className = "AngularGoalDataName"

  var myMaxAccelInput = document.createElement('INPUT');
  myMaxAccelInput.id = "AngularGoalDataMaxAccelInput" + x
  myMaxAccelInput.setAttribute("type", "text");
  myMaxAccelInput.className = "AngularGoalDataMaxAccelInput"
  myMaxAccelInput.onblur = function() {GoalAngularInput(x);}
  myMaxAccelInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalAngularInput(x);
    }
  }

  myMaxAccelDiv.appendChild(myMaxAccel);
  myMaxAccelDiv.appendChild(myMaxAccelInput);

  // 4
  var myTolOuterDiv = document.createElement('DIV');
  myTolOuterDiv.id = "AngularGoalDataTolOuterDiv" + x
  myTolOuterDiv.className = "AngularGoalDataDiv"

  var myTolOuter = document.createElement('H5');
  myTolOuter.id = "AngularGoalDataTolOuter" + x
  myTolOuter.textContent = "Tol Outer"
  myTolOuter.className = "AngularGoalDataName"

  var myTolOuterInput = document.createElement('INPUT');
  myTolOuterInput.id = "AngularGoalDataTolOuterInput" + x
  myTolOuterInput.setAttribute("type", "text");
  myTolOuterInput.className = "AngularGoalDataTolOuterInput"
  myTolOuterInput.onblur = function() {GoalAngularInput(x);}
  myTolOuterInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalAngularInput(x);
    }
  }

  myTolOuterDiv.appendChild(myTolOuter);
  myTolOuterDiv.appendChild(myTolOuterInput);

  // 5
  var myTolInnerDiv = document.createElement('DIV');
  myTolInnerDiv.id = "AngularGoalDataTolInnerDiv" + x
  myTolInnerDiv.className = "AngularGoalDataDiv"

  var myTolInner = document.createElement('H5');
  myTolInner.id = "AngularGoalDataTolInner" + x
  myTolInner.textContent = "Tol Inner"
  myTolInner.className = "AngularGoalDataName"

  var myTolInnerInput = document.createElement('INPUT');
  myTolInnerInput.id = "AngularGoalDataTolInnerInput" + x
  myTolInnerInput.setAttribute("type", "text");
  myTolInnerInput.className = "AngularGoalDataTolInnerInput"
  myTolInnerInput.onblur = function() {GoalAngularInput(x);}
  myTolInnerInput.onkeyup = function(event){
    if (event.keyCode === 13) {
      GoalAngularInput(x);
    }
  }

  myTolInnerDiv.appendChild(myTolInner);
  myTolInnerDiv.appendChild(myTolInnerInput);

  // 6
  var myIgnoreAngleDiv = document.createElement('DIV');
  myIgnoreAngleDiv.id = "AngularGoalDataIgnoreAngleDiv" + x
  myIgnoreAngleDiv.className = "AngularGoalDataDiv"

  var myIgnoreAngle = document.createElement('H5');
  myIgnoreAngle.id = "AngularGoalDataIgnoreAngle" + x
  myIgnoreAngle.textContent = "Ignore Angle"
  myIgnoreAngle.className = "AngularGoalDataName"

  var myIgnoreAngleInput = document.createElement('INPUT');
  myIgnoreAngleInput.id = "AngularGoalDataIgnoreAngleInput" + x
  myIgnoreAngleInput.setAttribute("type", "checkbox");
  myIgnoreAngleInput.className = "AngularGoalDataIgnoreAngleInput"
  myIgnoreAngleInput.onclick = function(){
    GoalAngularInput(x);
  }

  myIgnoreAngleDiv.appendChild(myIgnoreAngle);
  myIgnoreAngleDiv.appendChild(myIgnoreAngleInput);

  myDiv.appendChild(myMaxSpeedDiv); // 1
  myDiv.appendChild(myMinSpeedDiv); // 2
  myDiv.appendChild(myMaxAccelDiv); // 3
  myDiv.appendChild(myTolOuterDiv); // 4
  myDiv.appendChild(myTolInnerDiv); // 5
  myDiv.appendChild(myIgnoreAngleDiv); // 6


  myGoalContent.appendChild(myDropDown)
  myGoalContent.appendChild(myDiv)

  // Set default content for input

  //1
  if (autonData.paths[currentSelctedPathIndex].goals[x].max_angular_speed == null){
    document.getElementById("AngularGoalDataMaxSpeedInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("AngularGoalDataMaxSpeedInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].max_angular_speed;
  }

  //2
  if (autonData.paths[currentSelctedPathIndex].goals[x].min_angular_speed == null){
    document.getElementById("AngularGoalDataMinSpeedInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("AngularGoalDataMinSpeedInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].min_angular_speed;
  }

  //3
  if (autonData.paths[currentSelctedPathIndex].goals[x].max_angular_acceleration == null){
    document.getElementById("AngularGoalDataMaxAccelInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("AngularGoalDataMaxAccelInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].max_angular_acceleration;
  }

  //4
  if (autonData.paths[currentSelctedPathIndex].goals[x].angular_tolerance_outer == null){
    document.getElementById("AngularGoalDataTolOuterInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("AngularGoalDataTolOuterInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].angular_tolerance_outer;
  }

  //5
  if (autonData.paths[currentSelctedPathIndex].goals[x].angular_tolerance_inner == null){
    document.getElementById("AngularGoalDataTolInnerInput" + x).placeholder = "Select Value";
  }
  else{
    document.getElementById("AngularGoalDataTolInnerInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].angular_tolerance_inner;
  }

  //6
  document.getElementById("AngularGoalDataIgnoreAngleInput" + x).checked = autonData.paths[currentSelctedPathIndex].goals[x].ignore_angular_tolerance;
}

function GoalAngularInput(x){
  console.log("Angular is working");

  var inputValMaxSpeed = parseFloat(document.getElementById("AngularGoalDataMaxSpeedInput" + x).value);
  if (inputValMaxSpeed == ""){
    inputValMaxSpeed = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].max_angular_speed = inputValMaxSpeed

  var inputValMinSpeed = parseFloat(document.getElementById("AngularGoalDataMinSpeedInput" + x).value);
  if (inputValMinSpeed == ""){
    inputValMinSpeed = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].min_angular_speed = inputValMinSpeed

  var inputValMaxAccel = parseFloat(document.getElementById("AngularGoalDataMaxAccelInput" + x).value);
  if (inputValMaxAccel == ""){
    inputValMaxAccel = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].max_angular_acceleration = inputValMaxAccel

  var inputValTolOuter = parseFloat(document.getElementById("AngularGoalDataTolOuterInput" + x).value);
  if (inputValTolOuter == ""){
    inputValTolOuter = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].angular_tolerance_outer = inputValTolOuter

  var inputValTolInner = parseFloat(document.getElementById("AngularGoalDataTolInnerInput" + x).value);
  if (inputValTolInner == ""){
    inputValTolInner = null;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].angular_tolerance_inner = inputValTolInner

  var inputValIgnoreAng = document.getElementById("AngularGoalDataIgnoreAngleInput" + x).checked;
  console.log(inputValIgnoreAng)
  if (inputValIgnoreAng){
    inputValIgnoreAng = true;
  }
  else{
    inputValIgnoreAng = false;
  }
  autonData.paths[currentSelctedPathIndex].goals[x].ignore_angular_tolerance = inputValIgnoreAng

  postJSON();
}

function moveRobot(){
  myFieldWidth = document.getElementById('field').clientWidth
  myFieldHeight = document.getElementById('field').clientHeight
  console.log(myFieldWidth, myFieldHeight)
}

function setRobotStart(){

}

function resetRobot(){

}

//  Runs once at the beginning of the page loading
getAutonJSON('http://localhost:5000/planner/api/auton', getCallback)
setInterval(function(){ getAutonJSON('http://localhost:5000/planner/api/robot_pose', getPoseCallback); moveRobot();}, 100);

// Setting the default open tabs
document.getElementById("right_default_tab").click()
document.getElementById("left_default_tab").click()

// Prevent image from being dragged
document.getElementById('field').ondragstart = function() { return false; };

addEventListener('unload', (event) => {
  postJSON()
});