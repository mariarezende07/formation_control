for i=1:length(x_SS_leader(1,:))
    
    xlim([min(x_SS_5(1,:)), max(x_SS_2(1,:))]);
    ylim([min(x_SS_6(2,:)), max(x_SS_4(2,:))]);
    grid on;
    hold on;
    plot(x_SS_1(1,i),x_SS_1(2,i),'LineWidth',2);
    
    plot(x_SS_2(1,i),x_SS_2(2,i),'o','MarkerFaceColor','red');
    plot(x_SS_3(1,i),x_SS_3(2,i),'o','MarkerFaceColor','red');
    plot(x_SS_4(1,i),x_SS_4(2,i),'o','MarkerFaceColor','red');
    plot(x_SS_5(1,i),x_SS_5(2,i),'o','MarkerFaceColor','red');
    plot(x_SS_6(1,i),x_SS_6(2,i),'o','MarkerFaceColor','red');
    plot(x_SS_7(1,i),x_SS_7(2,i),'o','MarkerFaceColor','red');
    hold off
    drawnow
end