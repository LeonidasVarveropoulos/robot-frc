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

// For controlling the tab content
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

// Used for goal drop down reset
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