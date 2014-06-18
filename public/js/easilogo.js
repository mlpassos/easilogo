$(document).ready(function(){
    init('canvas','turtle','input','oldcode', 'textOutput'); 
    $('.controls .left, .controls .right').one('webkitAnimationEnd mozAnimationEnd MSAnimationEnd oanimationend animationend', function(){
    		$('.controls .left').removeClass("play");
    		$('.controls .right').removeClass("play-reverse");
	});
/*
 	$("#sendHardware").click(function(e){
        e.preventDefault();
        // logoCode when user hits send to hardware button
        // can do whatever want with it in here before sending to node via $.get
        // may transform the code to gcode and send to node
        // once there we can break it up and run everyline at once, waiting on an OK
        // answer back to go over the next line
        var logoCode = $("#code").val();
 	   if ($(this).hasClass("reset")) {
 		stop();
 		setup();
 		clearcanvas();
 		$('#turtle').css('transform', 'rotate(0deg');
 		$(this).removeClass('reset');
 		$(this).text('Run');
 		$('.controls .left').removeClass("play");
    	     $('.controls .right').removeClass("play-reverse");
 	   } else {
 		$(this).addClass('reset');
 		$(this).text('Reset');
 		$('.controls .left').addClass("play");
 		$('.controls .right').addClass("play-reverse");
           alert('Please center the Etch-a-Sketch and wipe it');
 		run(25,true);
 	   }
    });
*/
    $("#run").click(function(e){
 		e.preventDefault();
 		if ($(this).hasClass("reset")) {
 			stop();
 			setup();
 			clearcanvas();
 			$('#turtle').css('transform', 'rotate(0deg');
 			$(this).removeClass('reset');
 			$(this).text('Draw');
 			$('.controls .left').removeClass("play");
    		$('.controls .right').removeClass("play-reverse");
 		} else {
 			$(this).addClass('reset');
 			$(this).text('Reset');
 			$('.controls .left').addClass("play");
 			$('.controls .right').addClass("play-reverse");
 			run(25,true);
 		}
 	});
 	//$("#stop").click(function(e){
 	//	e.preventDefault();
     //   $('.controls .left').removeClass("play");
     //   $('.controls .right').removeClass("play-reverse");
 	//	stop();
 	//});
 	$("#clear").click(function(e){
 		e.preventDefault();
        //$('.controls .left').removeClass("play");
        //$('.controls .right').removeClass("play-reverse");
        $('.eas-box').addClass("clear");
    	$('.eas-box').one('webkitAnimationEnd mozAnimationEnd MSAnimationEnd oanimationend animationend', function(){
    		$('.eas-box').removeClass("clear");
	   	});
	   	clearcanvas();  
 	});
 	// clearcanvas(); 
 	// run(1,false);
});
