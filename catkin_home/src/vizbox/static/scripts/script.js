var ws = new WebSocket("ws://localhost:8888/ws");

ws.onmessage = function (evt) {
    var json = JSON.parse(evt.data);

    switch(json.label)
    {
        case "operator_text":
        case "robot_text":
            var subtitles = document.getElementById("subtitles");
            var txt = document.createElement("li");
            txt.appendChild(document.createTextNode(json.text));
            txt.setAttribute("class", json.label + " subtitle-line");
            console.log(json.label) // added line
            subtitles.insertAdjacentElement('afterbegin', txt);

            var footer = $(".footer");
            footer.animate({ scrollTop: footer.prop("scrollHeight") - footer.height() }, 100);
            break
        case "challenge_step":
            $old = $(".activated").removeClass('activated');

            $next_active = $('#storyline li').eq(json.index)
            $next_active.addClass('activated');
            break
        case "image":
            var image = "data:image/png;base64," + json.image;
            console.log("Got image: '"+image+"'")
            document.getElementById('visualization_img').setAttribute('src', image);
            break;
        case "story":
            $("#title").text(json.title)

            console.log("Got story: '"+json+"'")

            $("#storyline").empty();
            $(json.storyline).each(function(index){
                console.log("Got line: '"+this+"'")
                $("#storyline").append('<li>' + this + '</li>');
            });
            break;
    }
};

function send_to_ws() {
    var formdata = $( "#command_enter_form" ).serialize();
    ws.send(formdata);
}

$("#submit").on("click", send_to_ws);

$("#btn1").click(function () {
                $.ajax({
                    type: 'POST',
                    url: "/",
                    data : {'btn' : 1}
                });
            });


$("#btn2").click(function () {
    $.ajax({
        type: 'POST',
        url: "/",
        data : {'btn' : 2}
    });
});