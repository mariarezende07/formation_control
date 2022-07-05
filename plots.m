

%% Trajetória esperada
figure_1 = figure;
plot(x_SS_leader(1,:),x_SS_leader(2,:),'LineWidth',2);

hold on
plot(x_SS_2(1,:),x_SS_2(2,:),'LineWidth',2);
plot(x_SS_3(1,:),x_SS_3(2,:),'LineWidth',2);
plot(x_SS_4(1,:),x_SS_4(2,:),'LineWidth',2);
for i=1:100:1001
    plot([x_SS_leader(1,i),x_SS_3(1,i)],[x_SS_leader(2,i),x_SS_3(2,i)],'color','k','LineWidth',1);
    plot([x_SS_leader(1,i),x_SS_2(1,i)],[x_SS_leader(2,i),x_SS_2(2,i)],'color','k','LineWidth',1);

    plot([x_SS_2(1,i),x_SS_4(1,i)],[x_SS_2(2,i),x_SS_4(2,i)],'color','k','LineWidth',1);
    plot([x_SS_4(1,i),x_SS_3(1,i)],[x_SS_4(2,i),x_SS_3(2,i)],'color','k','LineWidth',1);


end
title("Trajetória esperada dos agentes no plano");

legend('Líder','seguidor 1', 'seguidor 2','seguidor 3','location','best');
xlabel('x(m)')
ylabel('y(m)')
axis equal
grid on
print(figure_1,'-dpng','-r300','trajetoria-ref_formacao-quadrado.png')
%% Trajetória real


figure_2 = figure;
plot(eta_1(1,:),eta_1(2,:),'LineWidth',2);

hold on
plot(eta_2(1,:),eta_2(2,:),'LineWidth',2);
plot(eta_3(1,:),eta_3(2,:),'LineWidth',2);
plot(eta_4(1,:),eta_4(2,:),'LineWidth',2);
for i=1:100:1001
    plot([eta_1(1,i),eta_3(1,i)],[eta_1(2,i),eta_3(2,i)],'color','k','LineWidth',1);
    plot([eta_1(1,i),eta_2(1,i)],[eta_1(2,i),eta_2(2,i)],'color','k','LineWidth',1);

    plot([eta_2(1,i),eta_4(1,i)],[eta_2(2,i),eta_4(2,i)],'color','k','LineWidth',1);
    plot([eta_4(1,i),eta_3(1,i)],[eta_4(2,i),eta_3(2,i)],'color','k','LineWidth',1);
end
title("Trajetória real dos agentes no plano");

legend('Líder','seguidor 1', 'seguidor 2','location','best');
xlabel('x(m)')
ylabel('y(m)')
axis equal
grid on
print(figure_2,'-dpng','-r300','trajetoria-real_formacao-quadrado.png')

%% Tau


figure_3 = figure;
plot(1:1000, tau(1,:),'LineWidth',2);

hold on
plot(1:1000, tau(2,:),'LineWidth',2);
plot(1:1000, tau(3,:),'LineWidth',2);

title("Força de controle");

xlabel('t(s)')
ylabel('tau(N)')
grid on
print(figure_3,'-dpng','-r300','tau_formacao-quadrado.png')

%% Q


figure_4 = figure;
subplot(3,1,1);
plot(1:1000, Q_2(1,1:1000),'LineWidth',2);

hold on
plot(1:1000, Q_2(2,1:1000),'LineWidth',2);
plot(1:1000, Q_2(3,1:1000),'LineWidth',2);

title("Forças de vínculo");

legend('f_u(N)','f_v(N)', 'n_r(Nm)','location','best');
xlabel('t(s)')
ylabel('')
grid on

subplot(3,1,2);
plot(1:1000, Q_3(1,1:1000),'LineWidth',2);

hold on
plot(1:1000, Q_3(2,1:1000),'LineWidth',2);
plot(1:1000, Q_3(3,1:1000),'LineWidth',2);

title("Forças de vínculo");

legend('f_u(N)','f_v(N)', 'n_r(Nm)','location','best');
xlabel('t(s)')
ylabel('')
grid on

subplot(3,1,1);
plot(1:1000, Q_4(1,1:1000),'LineWidth',2);

hold on
plot(1:1000, Q_4(2,1:1000),'LineWidth',2);
plot(1:1000, Q_4(3,1:1000),'LineWidth',2);

title("Forças de vínculo");

legend('f_u(N)','f_v(N)', 'n_r(Nm)','location','best');
xlabel('t(s)')
ylabel('')
grid on
print(figure_4,'-dpng','-r300','q_formacao-quadrado.png')
%% Deslocamento de referência pelo tempo


figure_5 = figure;

subplot(2,1,1)
plot(1:1000,x_SS_leader(1,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('x(m)')
grid on

subplot(2,1,2)
plot(1:1000,x_SS_leader(2,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('y(m)')
grid on

sgtitle('Deslocamento do líder') 

figure_6 = figure;

subplot(2,1,1)
plot(1:1000,x_SS_2(1,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('x(m)')
grid on
subplot(2,1,2)
plot(1:1000,x_SS_2(2,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('y(m)')
grid on

sgtitle('Deslocamento do agente 2') 

figure_7 = figure;

subplot(2,1,1)
plot(1:1000,x_SS_3(1,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('x(m)')
grid on

subplot(2,1,2)
plot(1:1000,x_SS_3(2,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('y(m)')
grid on
sgtitle('Deslocamento do agente 3') 


figure_8 = figure;

subplot(2,1,1)
plot(1:1000,x_SS_4(1,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('x(m)')
grid on

subplot(2,1,2)
plot(1:1000,x_SS_4(2,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('y(m)')
grid on
sgtitle('Deslocamento do agente 3') 

print(figure_5,'-dpng','-r300','x1xt_formacao-quadrado.png')
print(figure_6,'-dpng','-r300','x2xt_formacao-quadrado.png')
print(figure_7,'-dpng','-r300','x2xt_formacao-quadrado.png')
print(figure_8,'-dpng','-r300','x2xt_formacao-quadrado.png')

%% Deslocamento de real pelo tempo


figure_8 = figure;

subplot(2,1,1)
plot(1:1000,eta_1(1,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('x(m)')
grid on

subplot(2,1,2)
plot(1:1000,eta_1(2,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('y(m)')
grid on

sgtitle('Deslocamento do líder') 

figure_9 = figure;

subplot(2,1,1)
plot(1:1000,eta_2(1,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('x(m)')
grid on
subplot(2,1,2)
plot(1:1000,eta_2(2,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('y(m)')
grid on

sgtitle('Deslocamento do agente 2') 

figure_10 = figure;

subplot(2,1,1)
plot(1:1000,eta_3(1,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('x(m)')
grid on

subplot(2,1,2)
plot(1:1000,eta_3(2,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('y(m)')
grid on
sgtitle('Deslocamento do agente 3') 

figure_10 = figure;

subplot(2,1,1)
plot(1:1000,eta_4(1,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('x(m)')
grid on

subplot(2,1,2)
plot(1:1000,eta_4(2,1:1000),'LineWidth',2);
xlabel('t(s)')
ylabel('y(m)')
grid on
sgtitle('Deslocamento do agente 4') 

print(figure_9,'-dpng','-r300','e1xt_formacao-quadrado.png')
print(figure_10,'-dpng','-r300','e2xt_formacao-quadrado.png')
print(figure_11,'-dpng','-r300','e3xt_formacao-quadrado.png')
print(figure_12,'-dpng','-r300','e3xt_formacao-quadrado.png')