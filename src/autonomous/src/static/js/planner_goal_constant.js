// This displys the constants of the goal data in a UI
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

// This takes care of the input of the constants which update the main data
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