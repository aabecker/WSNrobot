
 h1=1e3*[    9.5116
    9.0450
    8.6449
    8.3084
    8.0238
    7.7824
    7.5761
    7.3996
    7.2449
    7.1090
    6.9887
    6.8812
    6.7890
    6.7093
    6.6424
    6.5833
    6.5324
    6.4891
    6.4497
    6.4165
    6.3845
    6.3585
    6.3339
    6.3132
    6.2941
    6.2754
    6.2612
    6.2465
    6.2332
    6.2222
    6.2120
    6.2022
    6.1938
    6.1851
    6.1753
    6.1654
    6.1572
    6.1480
    6.1387
    6.1301
    6.1212
    6.1103
    6.0980
    6.0870
    6.0762
    6.0652
    6.0542
    6.0444
    6.0350
    6.0237
    6.0111
    6.0003
    5.9890
    5.9762
    5.9647
    5.9521
    5.9411
    5.9294
    5.9163
    5.9041
    5.8922
    5.8803
    5.8694
    5.8570
    5.8449
    5.8330
    5.8194
    5.8078
    5.7958
    5.7816
    5.7713
    5.7575
    5.7461
    5.7344
    5.7228
    5.7129
    5.7039
    5.6946
    5.6878
    5.6820
    5.6774
    5.6721
    5.6660
    5.6626
    5.6584
    5.6540
    5.6505
    5.6464
    5.6428
    5.6401
    5.6338
    5.6293
    5.6259
    5.6221
    5.6188
    5.6152
    5.6091
    5.6048
    5.6005
    5.5971
    5.5934]';
h2=1e3*[      4.7669
    4.7277
    4.6926
    4.6619
    4.6349
    4.6109
    4.5893
    4.5695
    4.5517
    4.5356
    4.5210
    4.5071
    4.4947
    4.4834
    4.4728
    4.4636
    4.4547
    4.4458
    4.4379
    4.4311
    4.4234
    4.4168
    4.4105
    4.4044
    4.3988
    4.3934
    4.3884
    4.3831
    4.3783
    4.3740
    4.3702
    4.3665
    4.3624
    4.3574
    4.3514
    4.3427
    4.3340
    4.3279
    4.3240
    4.3202
    4.3166
    4.3136
    4.3107
    4.3077
    4.3049
    4.3020
    4.2994
    4.2966
    4.2944
    4.2921
    4.2901
    4.2881
    4.2865
    4.2850
    4.2836
    4.2823
    4.2808
    4.2792
    4.2780
    4.2767
    4.2754
    4.2742
    4.2729
    4.2715
    4.2703
    4.2689
    4.2678
    4.2666
    4.2656
    4.2648
    4.2640
    4.2631
    4.2622
    4.2613
    4.2605
    4.2598
    4.2591
    4.2584
    4.2577
    4.2570
    4.2563
    4.2554
    4.2545
    4.2537
    4.2530
    4.2521
    4.2514
    4.2509
    4.2504
    4.2499
    4.2494
    4.2489
    4.2484
    4.2471
    4.2466
    4.2460
    4.2455
    4.2451
    4.2448
    4.2444
    4.2441]';

h3=1e3*[4.2033];

plot(0:100,h1,'b','linewidth',2);
hold on;
plot([100,200],[h1(end),h2(1)],'k-o','linewidth',2);
hold on;
plot(200:300,h2,'r','linewidth',2);
hold on;
% plot([300,400],[h2(end),h3(1)],'k-o','linewidth',2);
% hold on;
% plot(400:500,h3,'g','linewidth',2);
% hold on;
 title 'Cost Function for Multi Robot Case';
xlabel 'Iteration Number';
ylabel 'Cost Function';
legend ('Gradient Descent-1','mTSP','Gradient Descent-2');
set(gca,'xtick',[1,50,100,200,250,300], 'xticklabel',{'1','50','100','1','50','100'});