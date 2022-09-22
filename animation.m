
myVideo = VideoWriter('formacao_triangulo'); %open video file
myVideo.FrameRate = 5;  %can adjust this, 5 - 10 works well for me
open(myVideo)
a = animatedline('Color','r');
b = animatedline('Color','g');
c = animatedline('Color','b');

for i=1:9:length(x_SS_leader(1,:))
    
    xlim([min(x_SS_2(1,:)), max(x_SS_1(1,:))]);
    ylim([min(x_SS_2(2,:)), max(x_SS_2(2,:))]);
    grid on;
    addpoints(a,x_SS_1(1,i),x_SS_1(2,i));    
    addpoints(b,x_SS_2(1,i),x_SS_2(2,i));
    addpoints(c,x_SS_3(1,i),x_SS_3(2,i));
    
    if rem(i,10) == 0
        hold on
        plot([x_SS_2(1,i),x_SS_3(1,i)],[x_SS_2(2,i),x_SS_3(2,i)],'color','k','LineWidth',1);
        plot([x_SS_leader(1,i),x_SS_3(1,i)],[x_SS_leader(2,i),x_SS_3(2,i)],'color','k','LineWidth',1);
        plot([x_SS_leader(1,i),x_SS_2(1,i)],[x_SS_leader(2,i),x_SS_2(2,i)],'color','k','LineWidth',1);
        hold off
    end
    pause(0.1) %Pause and grab frame
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    
end
close(myVideo)
