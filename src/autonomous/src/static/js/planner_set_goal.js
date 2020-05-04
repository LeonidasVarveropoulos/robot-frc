var isBorder = false;

function setGoal(e){
    var myFieldWidth = document.getElementById('field').clientWidth;
    var myFieldHeight = document.getElementById('field').clientHeight;

    // In meters
    var widthRatio = (16.4846)/myFieldWidth;
    var heightRatio = 8.1026/myFieldHeight;

    var rect = e.target.getBoundingClientRect();
    var x = e.clientX - rect.left; //x position within the element.
    var y = e.clientY - rect.top;  //y position within the element.
    
    var xDistance = widthRatio * x;
    var yDistance = heightRatio * y;
    console.log(xDistance, yDistance)

}

function createFieldBorder(){
    var myFieldWidth = document.getElementById('field').clientWidth;
    var myFieldHeight = document.getElementById('field').clientHeight;

    if (myFieldHeight > 100 && myFieldWidth > 100){
        isBorder = true;
    }
    var width = "width:" + myFieldWidth + "px";
    var height = "height:" + myFieldHeight + "px";
    var style = width + ";" + height + ";";

    document.getElementById('field_border').setAttribute("style",style);
}

createFieldBorder()
if (!isBorder){
    setInterval(createFieldBorder, 100);
}
    