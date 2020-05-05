// This displays the angular data of the goal in a UI
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

  updateDefaultAngular(x);
}

function updateDefaultAngular(x){
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

// This takes the input and updates the main angular data
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