// This displays the goal pose data in the UI
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

  updateDefaultPose(x)
  
}

function updateDefaultPose(x){
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

  if (autonData.paths[currentSelctedPathIndex].goals[x].th == null){
    document.getElementById("PoseGoalDataThInput" + x).placeholder = "Select Th Pose";
  }
  else{
    document.getElementById("PoseGoalDataThInput" + x).defaultValue = autonData.paths[currentSelctedPathIndex].goals[x].th;
  }
}

// This manages the input to changing the goal data
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