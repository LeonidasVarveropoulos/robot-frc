//  Runs once at the beginning of the page loading
getAutonJSON('http://127.0.0.1:5000/planner/api/auton', getCallback)

// Setting the default open tabs
document.getElementById("right_default_tab").click()
document.getElementById("left_default_tab").click()

// Prevent image from being dragged
document.getElementById('field').ondragstart = function() { return false; };

// When closed it updates the json file
addEventListener('unload', (event) => {
  postJSON()
});