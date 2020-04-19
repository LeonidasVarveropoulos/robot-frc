function noneTypeAlert()
{
    var a = document.getElementById("noneAlert").value;
    var boolValue = (/True/i).test(a)
    if (boolValue === true)
    {
       alert("Error in selecting autonomous please try again!");
    }
        
}