// PATHS

// This adds a new path to the stored data
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

// Manages the input to creating a new path
function addPathInput(){
  var inputVal = document.getElementById("myPathInput").value;
  console.log("Created a new path")
  autonData.paths.push({"id":autonData.paths.length, "goals":[] , "name":inputVal})
  console.log(autonData)
  postJSON();
  displayPathTable();
}

// Deletes a path from the stored data
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

    currentSelctedGoalIndex = null
  }
  else{
    document.getElementById(id).classList.toggle('active')
    currentSelectedPath = autonData.paths[id.slice(5,id.length)]
    currentSelctedPathIndex = id.slice(5,id.length)
    console.log(currentSelectedPath)

    currentSelctedGoalIndex = null
  }
  displayGoalDropDown()
  
}