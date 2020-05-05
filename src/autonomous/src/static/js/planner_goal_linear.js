// Displays the goal linear section of the data in a UI
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

  updateDefaultLinear(x)
}

function updateDefaultLinear(x){
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

// This manages the input to changing the linear goal data
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