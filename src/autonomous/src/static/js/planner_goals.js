// GOALS

// This adds a goal to the data set
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

// This deletes a goal from the data
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

// This displays the goal data for the selected path
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

// This displays the UI for editing values of the goal
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