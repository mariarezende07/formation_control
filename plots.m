

%% Trajetória esperada
figure_1 = figure;
plot(x_SS_1(1,:),x_SS_1(2,:),'LineWidth',2);

hold on
plot(x_SS_2(1,:),x_SS_2(2,:),'LineWidth',2);
plot(x_SS_3(1,:),x_SS_3(2,:),'LineWidth',2);
plot(x_SS_4(1,:),x_SS_4(2,:),'LineWidth',2);
plot(x_SS_5(1,:),x_SS_5(2,:),'LineWidth',2);
plot(x_SS_6(1,:),x_SS_6(2,:),'LineWidth',2);
plot(x_SS_7(1,:),x_SS_7(2,:),'LineWidth',2);
for i=1:10:101
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
print(figure_1,'-dpng','-r300','figuras/trajetoria-ref_formacao-hexagonal.png')
%% Trajetória real


figure_2 = figure;
plot(eta_2(1,:),eta_2(2,:),'LineWidth',2);
hold on
plot(eta_3(1,:),eta_3(2,:),'LineWidth',2);
plot(eta_4(1,:),eta_4(2,:),'LineWidth',2);
plot(eta_5(1,:),eta_5(2,:),'LineWidth',2);
plot(eta_6(1,:),eta_6(2,:),'LineWidth',2);
plot(eta_7(1,:),eta_7(2,:),'LineWidth',2);
for i=1:100:1001
    plot([eta_1(1,i),eta_3(1,i)],[eta_1(2,i),eta_3(2,i)],'color','k','LineWidth',1);
    plot([eta_1(1,i),eta_2(1,i)],[eta_1(2,i),eta_2(2,i)],'color','k','LineWidth',1);

    plot([eta_2(1,i),eta_4(1,i)],[eta_2(2,i),eta_4(2,i)],'color','k','LineWidth',1);
    plot([eta_4(1,i),eta_3(1,i)],[eta_4(2,i),eta_3(2,i)],'color','k','LineWidth',1);
end
title("Trajetória real dos agentes no plano");

legend('Líder','seguidor 1', 'seguidor 2','seguidor 3','location','best');
xlabel('x(m)')
ylabel('y(m)')
axis equal
grid on
print(figure_2,'-dpng','-r300','figuras/trajetoria-real_formacao-hexagonal.png')

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
print(figure_3,'-dpng','-r300','figuras/tau_formacao-hexagonal.png')

%% Q


figure_4 = figure;
subplot(2,1,2);

plot(1:1000, Q_2(1,1:1000),'LineWidth',2);

hold on
plot(1:1000, Q_2(2,1:1000),'LineWidth',2);
plot(1:1000, Q_2(3,1:1000),'LineWidth',2);

title("Forças de vínculo no seguidor 2");

legend('f_u(N)','f_v(N)', 'n_r(Nm)','location','best');
xlabel('t(s)')
ylabel('')
grid on

subplot(2,1,2);
plot(1:1000, Q_3(1,1:1000),'LineWidth',2);

hold on
plot(1:1000, Q_3(2,1:1000),'LineWidth',2);
plot(1:1000, Q_3(3,1:1000),'LineWidth',2);

title("Forças de vínculo no seguidor 3");

legend('f_u(N)','f_v(N)', 'n_r(Nm)','location','best');
xlabel('t(s)')
ylabel('')
grid on

subplot(2,1,1);
plot(1:1000, Q_4(1,1:1000),'LineWidth',2);

hold on
plot(1:1000, Q_4(2,1:1000),'LineWidth',2);
plot(1:1000, Q_4(3,1:1000),'LineWidth',2);

title("Forças de vínculo no seguidor 4");

legend('f_u(N)','f_v(N)', 'n_r(Nm)','location','best');
xlabel('t(s)')
ylabel('')
grid on

subplot(2,1,2);
plot(1:1000, Q_5(1,1:1000),'LineWidth',2);

hold on
plot(1:1000, Q_5(2,1:1000),'LineWidth',2);
plot(1:1000, Q_5(3,1:1000),'LineWidth',2);

title("Forças de vínculo no seguidor 3");

legend('f_u(N)','f_v(N)', 'n_r(Nm)','location','best');
xlabel('t(s)')
ylabel('')
grid on

subplot(2,1,2);
plot(1:1000, Q_6(1,1:1000),'LineWidth',2);

hold on
plot(1:1000, Q_6(2,1:1000),'LineWidth',2);
plot(1:1000, Q_6(3,1:1000),'LineWidth',2);

title("Forças de vínculo no seguidor 3");

legend('f_u(N)','f_v(N)', 'n_r(Nm)','location','best');
xlabel('t(s)')
ylabel('')
grid on

subplot(2,1,2);
plot(1:1000, Q_7(1,1:1000),'LineWidth',2);

hold on
plot(1:1000, Q_7(2,1:1000),'LineWidth',2);
plot(1:1000, Q_7(3,1:1000),'LineWidth',2);

title("Forças de vínculo no seguidor 3");

legend('f_u(N)','f_v(N)', 'n_r(Nm)','location','best');
xlabel('t(s)')
ylabel('')
grid on
print(figure_4,'-dpng','-r300','figuras/q_formacao-hexagonal.png')
%% Deslocamento de referência pelo tempo


figure_5 = figure;

subplot(2,1,1)
plot(1:1000,x_SS_leader(1,1:1000),'LineWidth',2);
hold on
plot(1:1000,x_SS_2(1,1:1000),'LineWidth',2);
plot(1:1000,x_SS_3(1,1:1000),'LineWidth',2);
plot(1:1000,x_SS_4(1,1:1000),'LineWidth',2);
plot(1:1000,x_SS_5(1,1:1000),'LineWidth',2);
plot(1:1000,x_SS_6(1,1:1000),'LineWidth',2);
plot(1:1000,x_SS_7(1,1:1000),'LineWidth',2);


xlabel('t(s)')
ylabel('x(m)')
legend('Líder','seguidor 1', 'seguidor 2','location','best');

grid on

subplot(2,1,2)
plot(1:1000,x_SS_leader(2,1:1000),'LineWidth',2);
hold on
plot(1:1000,x_SS_2(2,1:1000),'LineWidth',2);
plot(1:1000,x_SS_3(2,1:1000),'LineWidth',2);
plot(1:1000,x_SS_4(2,1:1000),'LineWidth',2);
plot(1:1000,x_SS_5(2,1:1000),'LineWidth',2);
plot(1:1000,x_SS_6(2,1:1000),'LineWidth',2);
plot(1:1000,x_SS_7(2,1:1000),'LineWidth',2);

xlabel('t(s)')
ylabel('y(m)')
legend('Líder','seguidor 1', 'seguidor 2','location','best');

grid on

sgtitle('Deslocamento dos agentes em relação ao tempo') 


print(figure_5,'-dpng','-r300','figuras/x1xt_formacao-hexagonal.png')

%% Deslocamento de real pelo tempo


figure_6 = figure;

subplot(2,1,1)
plot(1:1000,eta_1(1,1:1000),'LineWidth',2);
hold on
plot(1:1000,eta_2(1,1:1000),'LineWidth',2);
plot(1:1000,eta_3(1,1:1000),'LineWidth',2);

plot(1:1000,eta_4(1,1:1000),'LineWidth',2);
plot(1:1000,eta_5(1,1:1000),'LineWidth',2);
plot(1:1000,eta_6(1,1:1000),'LineWidth',2);
plot(1:1000,eta_7(1,1:1000),'LineWidth',2);

xlabel('t(s)')
ylabel('x(m)')
legend('Líder','seguidor 1', 'seguidor 2','seguidor 3','location','best');

grid on

subplot(2,1,2)
plot(1:1000,eta_1(2,1:1000),'LineWidth',2);
hold on
plot(1:1000,eta_2(2,1:1000),'LineWidth',2);
plot(1:1000,eta_3(2,1:1000),'LineWidth',2);
plot(1:1000,eta_4(2,1:1000),'LineWidth',2);
plot(1:1000,eta_5(2,1:1000),'LineWidth',2);

plot(1:1000,eta_6(2,1:1000),'LineWidth',2);
plot(1:1000,eta_7(2,1:1000),'LineWidth',2);

xlabel('t(s)')
ylabel('y(m)')
legend('Líder','seguidor 1', 'seguidor 2','seguidor 3','location','best');

grid on

sgtitle('Deslocamento real em relação ao tempo') 


print(figure_6,'-dpng','-r300','figuras/e1xt_formacao-hexagonal.png')