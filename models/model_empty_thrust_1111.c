
#include "network_evaluate_tof.h"
#include <string.h>


#define EPS 0.000001 // 1e-6

static float obstacle_embeds[4];
static float output_embeds[20];

float base;
float exponent;
static const int self_structure [4][2] = {{18, 16},{16, 16},{20, 20},{20, 4}};
static const int obst_structure [2][2] = {{16, 4},{4, 4}};
static float output_0[16];
static float output_1[16];
static float obst_output_0[4];
static float obst_output_1[4];
static float output_2[20];
static float output_3[4];
static const float actor_encoder_self_encoder_0_weight[18][16] = {{-0.5550323724746704,0.4086778163909912,-0.20222263038158417,0.051499899476766586,-0.018314074724912643,-0.1480313092470169,0.10061919689178467,-0.045411597937345505,0.07871711999177933,0.26470956206321716,0.006738933734595776,0.11599534004926682,-0.1711604744195938,0.09382420778274536,-0.23776067793369293,0.10223156213760376},{-0.3833111822605133,-0.025012075901031494,-0.04275340214371681,0.2774891257286072,-0.0727333277463913,0.5156456232070923,-0.1345856785774231,0.08490601181983948,0.14928659796714783,-0.287201851606369,0.07085904479026794,0.4331081509590149,0.3031926453113556,-0.1489218920469284,0.14194312691688538,-0.06704312562942505},{-0.16156189143657684,-0.06942671537399292,-0.26992031931877136,-0.23441851139068604,0.30387550592422485,-0.16454562544822693,-0.8379613757133484,0.03535769134759903,0.024589527398347855,0.5355271697044373,-0.1511472463607788,0.13992029428482056,0.010032156482338905,0.4810987710952759,0.5125040411949158,-0.2509097456932068},{-0.3608725368976593,0.3255230784416199,-0.040462199598550797,-0.024211809039115906,0.20632752776145935,0.12790732085704803,0.038217198103666306,-0.007399418856948614,0.29121294617652893,0.07595828175544739,-0.012912417761981487,-0.0062951622530817986,-0.16474150121212006,-0.1819806694984436,-0.11691272258758545,0.04564843699336052},{-0.2104242444038391,0.07569188624620438,-0.21208752691745758,0.027012130245566368,-0.23742954432964325,0.06978993862867355,-0.09169137477874756,-0.17310675978660583,0.05859335511922836,-0.046212904155254364,-0.014562278985977173,0.314778596162796,-0.30338671803474426,-0.17847803235054016,0.013327684253454208,0.3332166075706482},{0.10886482149362564,0.14905789494514465,-0.39647409319877625,-0.19961966574192047,0.18592222034931183,0.009681192226707935,-0.4128933548927307,0.18727253377437592,0.39694279432296753,0.023583076894283295,-0.0057556224055588245,0.0961112305521965,-0.2125963419675827,0.32241714000701904,0.03397626057267189,-0.15843075513839722},{0.1411072015762329,-0.088630311191082,-0.37157753109931946,0.44625887274742126,-0.10281679034233093,-0.267525851726532,-0.24840374290943146,-0.16816146671772003,-0.45265188813209534,-0.274483859539032,-0.35610172152519226,-0.04728108271956444,0.1306021511554718,0.2603565752506256,-0.10263446718454361,-0.10110867023468018},{0.35733917355537415,0.2598448395729065,-0.43966808915138245,-0.02628275565803051,-0.2653505802154541,0.42423883080482483,-0.2055896520614624,0.6377898454666138,-0.3933279812335968,0.25229838490486145,0.045931726694107056,0.019404921680688858,-0.14931149780750275,-0.0763699933886528,0.142673060297966,0.5852813720703125},{-0.6578850150108337,0.07364233583211899,0.07360231131315231,0.7230372428894043,0.5107833743095398,0.2326919138431549,0.1771528571844101,0.18175601959228516,0.5846738815307617,0.2512998580932617,-0.5636283755302429,-0.11243818700313568,0.12062257528305054,-0.39755484461784363,0.31380152702331543,-0.02228863537311554},{-0.30862268805503845,0.2612631618976593,0.15124042332172394,-0.05746407434344292,0.04722428694367409,-0.27288129925727844,0.10476208478212357,-0.3722905218601227,-0.04259249567985535,-0.23103676736354828,-0.149773508310318,0.21872176229953766,0.3062148988246918,0.21725039184093475,-0.3740960955619812,-0.666235089302063},{0.45447027683258057,-0.16011853516101837,-0.6413841247558594,0.5463179349899292,0.2653084993362427,0.010005610063672066,0.31055599451065063,-0.2929683029651642,-0.39756566286087036,0.07169553637504578,-0.3625778257846832,0.02902737632393837,-0.2524639666080475,-0.39439937472343445,0.09320837259292603,0.35031720995903015},{-0.552983820438385,-0.1087794229388237,-0.3063982427120209,-0.38712581992149353,0.07523415982723236,0.4267379641532898,0.061206210404634476,-0.5792819857597351,-0.48530733585357666,-0.2627379894256592,-0.6978476643562317,0.8393985033035278,-0.0736025869846344,0.010019912384450436,0.6371838450431824,0.1729387640953064},{-0.5155430436134338,0.29186445474624634,0.12366768717765808,0.3035884201526642,0.09077353030443192,0.33355116844177246,0.05728935822844505,0.2460535615682602,-0.007960746064782143,-0.7761335372924805,-0.3111639618873596,-0.11242938041687012,0.30893221497535706,-0.3888779878616333,0.13382896780967712,0.26302486658096313},{0.33381372690200806,0.26853543519973755,-0.2892891764640808,0.2183476686477661,-0.23549054563045502,-0.06478649377822876,0.12597042322158813,-0.3457743227481842,-0.20712734758853912,-0.6339385509490967,-0.2006797194480896,0.7374855875968933,-0.09507614374160767,0.11604328453540802,0.2702246904373169,0.05516554415225983},{-0.36996909976005554,-0.24110561609268188,-0.1647941619157791,-0.3087713420391083,-0.11308375000953674,-0.5141631960868835,0.18640346825122833,0.3933072090148926,-0.4551338851451874,0.354234516620636,-0.1340947151184082,-0.10344606637954712,-0.21639391779899597,0.41360366344451904,0.2868055999279022,0.25373178720474243},{-0.006019384600222111,0.05440323054790497,-0.1794966459274292,-0.008160869590938091,-0.21724633872509003,-0.22026066482067108,-0.05037274956703186,-0.26947104930877686,-0.2038707584142685,-0.01189102977514267,0.05294741317629814,0.20980501174926758,0.3835236430168152,0.32019320130348206,0.0030404238495975733,0.11491452157497406},{0.1548544466495514,-0.26790767908096313,0.1078697219491005,-0.16138143837451935,-0.0980304405093193,0.18654519319534302,0.16771957278251648,0.076046884059906,-0.25287920236587524,0.3149838447570801,0.09544940292835236,-0.039223749190568924,-0.26381802558898926,0.2145862728357315,0.10960038751363754,-0.08417177945375443},{0.24914897978305817,0.13601715862751007,0.034376613795757294,0.08001568168401718,-0.08875031024217606,0.21174483001232147,-0.022362686693668365,0.14012350142002106,-0.1589324176311493,0.34523889422416687,0.08002689480781555,0.2864914536476135,-0.41175681352615356,-0.08474663645029068,0.03890440985560417,0.1946326643228531}};
static const float actor_encoder_self_encoder_2_weight[16][16] = {{-0.012410073541104794,0.1741200089454651,-0.1253960132598877,-0.0352180078625679,-0.23179566860198975,-0.28224024176597595,0.4141458570957184,-0.25540244579315186,-0.40649279952049255,-0.1556791514158249,-0.03708130121231079,-0.13504870235919952,-0.14136642217636108,0.22914494574069977,-0.20995965600013733,0.21353767812252045},{-0.22505299746990204,0.132692351937294,0.10168939083814621,-0.02315647155046463,-0.2704814374446869,0.01805185340344906,-0.3804793059825897,0.3090052902698517,0.3451252281665802,-0.11340770870447159,0.10442378371953964,0.26358211040496826,0.16386568546295166,0.3992452919483185,-0.34595534205436707,-0.5110597014427185},{0.19702906906604767,0.1899411380290985,0.335553377866745,-0.053075145930051804,-0.07309123873710632,-0.1636095494031906,0.16085726022720337,-0.2593122720718384,0.43614301085472107,0.3395775258541107,0.3803781270980835,-0.2454206645488739,0.026453135535120964,-0.4346020519733429,-0.134648397564888,0.23617856204509735},{-0.07742035388946533,0.057045530527830124,-0.04358428716659546,0.24052400887012482,-0.2450094074010849,0.3694665729999542,0.0510452575981617,0.3727608621120453,-0.44613510370254517,0.0497380867600441,-0.18331623077392578,0.10248005390167236,0.4441865086555481,0.160446435213089,0.16943199932575226,0.36513862013816833},{-0.20845262706279755,0.2296445518732071,0.03070790320634842,0.4332042932510376,0.20820456743240356,0.31605085730552673,0.10892968624830246,-0.5650314688682556,-0.06125585362315178,-0.13050726056098938,0.13642476499080658,0.14279323816299438,0.06881160289049149,0.14048992097377777,0.12697216868400574,0.1324480175971985},{0.14797642827033997,-0.02431114763021469,-0.2625424265861511,-0.4127326011657715,-0.3383350074291229,0.0074790590442717075,0.10957816988229752,0.08074473589658737,0.06813250482082367,-0.3153233528137207,-0.26601868867874146,0.3536985218524933,0.1065705344080925,0.23911863565444946,0.25833451747894287,-0.26786190271377563},{-0.17192134261131287,0.3865969479084015,-0.08169323205947876,0.1182883232831955,-0.028869004920125008,0.2775506377220154,-0.2736566960811615,0.22536081075668335,0.13754986226558685,0.4990631341934204,-0.11803372949361801,0.33299189805984497,-0.14703908562660217,0.4334321916103363,0.38591986894607544,0.25146228075027466},{-0.0658876895904541,-0.2808380126953125,-0.33025410771369934,0.477457731962204,-0.3544836938381195,0.48756298422813416,-0.37749767303466797,0.28133904933929443,-0.3577336370944977,0.08708207309246063,-0.0997016504406929,-0.4904929995536804,-0.36249586939811707,0.28295066952705383,0.06757913529872894,0.1966400146484375},{0.3324292302131653,-0.22294677793979645,0.3206169903278351,-0.17423762381076813,0.4714754819869995,-0.35852664709091187,-0.33056193590164185,0.141212597489357,0.18086963891983032,0.24791166186332703,0.20494286715984344,-0.3952118754386902,0.03565604239702225,0.16885337233543396,0.03749999776482582,-0.36079323291778564},{0.0076400660909712315,-0.2868339717388153,0.3715975880622864,0.37476250529289246,-0.1474483162164688,-0.19327038526535034,-0.44762754440307617,0.2546495199203491,-0.19971109926700592,0.1855412870645523,0.09921551495790482,-0.4868033826351166,-0.10161759704351425,-0.2680874466896057,-0.1856928914785385,0.038540855050086975},{0.21007812023162842,-0.2548237442970276,0.019940631464123726,-0.4165341258049011,-0.3328350782394409,-0.060760680586099625,-0.4498525857925415,0.3584955632686615,-0.13776443898677826,0.32880985736846924,0.17774716019630432,-0.3533281683921814,-0.440317839384079,0.1495850682258606,0.30762505531311035,-0.012801195494830608},{-0.3442409336566925,-0.10620959848165512,-0.013010849244892597,-0.14706310629844666,0.07820799946784973,-0.2320544421672821,-0.01971716247498989,0.4041772186756134,-0.03227278217673302,-0.1907360702753067,-0.4110005795955658,-0.3195977210998535,-0.1852690726518631,0.3195270299911499,-0.016350511461496353,-0.17969126999378204},{0.356976717710495,-0.3676929175853729,-0.02568873204290867,0.023714201524853706,-0.26807108521461487,0.18574769794940948,-0.04138609394431114,0.2867737412452698,0.24436146020889282,0.06812732666730881,-0.08326493948698044,-0.16974271833896637,0.044284749776124954,0.1460622251033783,-0.3990054726600647,0.3132246434688568},{-0.32244178652763367,0.04163702204823494,0.37274739146232605,0.2530515491962433,-0.16416482627391815,-0.4627792537212372,-0.2382575422525406,-0.31100115180015564,0.14747627079486847,0.1519768238067627,-0.1807170808315277,0.09496121108531952,-0.0016226511215791106,-0.06967290490865707,0.13612796366214752,0.09372584521770477},{-0.12211710959672928,-0.4353061020374298,-0.5301660895347595,0.44379156827926636,0.169863760471344,-0.08632197976112366,-0.23639288544654846,0.32553330063819885,0.4752277135848999,0.29082903265953064,-0.27949902415275574,0.05419829860329628,0.08586207777261734,0.011965487152338028,-0.43012598156929016,-0.31005775928497314},{0.1020968109369278,0.36549270153045654,-0.19287821650505066,0.29162153601646423,0.09146347641944885,0.2215849757194519,0.38592007756233215,0.008553852327167988,0.3909793794155121,-0.07772769033908844,-0.34841188788414,-0.043219152837991714,-0.11229445785284042,-0.20521382987499237,-0.3952164947986603,-0.25410786271095276}};
static const float actor_encoder_feed_forward_0_weight[20][20] = {{0.09887503832578659,0.07509448379278183,0.18939688801765442,0.4201401174068451,-0.04331761971116066,0.361423134803772,-0.19245277345180511,0.18955524265766144,0.06773582100868225,-0.02480708248913288,0.12782824039459229,-0.03980907425284386,0.047482483088970184,-0.246935173869133,-0.2763625681400299,0.008838018402457237,-0.1403200328350067,-0.1375754326581955,0.08141213655471802,0.26726290583610535},{0.12091850489377975,0.188642218708992,0.05564025044441223,0.4707518219947815,-0.2960682213306427,0.38898608088493347,0.4197053611278534,0.38443952798843384,-0.1426723748445511,-0.44394081830978394,-0.3514660596847534,0.03086923435330391,-0.07560106366872787,0.3200855553150177,-0.30345600843429565,-0.15885908901691437,0.08656801283359528,0.4526391327381134,-0.14767953753471375,0.176331028342247},{0.20805741846561432,0.07127612829208374,0.1763094961643219,0.024451259523630142,0.0008695150609128177,-0.061613529920578,0.005037763621658087,-0.2933562994003296,0.3213062882423401,0.12059655785560608,-0.32609236240386963,-0.1391531527042389,0.20023100078105927,-0.027648797258734703,0.17253781855106354,-0.3905135989189148,0.16675063967704773,0.2894919812679291,-0.0632433071732521,-0.40307602286338806},{0.23393917083740234,-0.02315765991806984,-0.14418694376945496,-0.49408209323883057,-0.26132920384407043,-0.4643736183643341,-0.21837876737117767,-0.3167840838432312,-0.23413681983947754,0.2557029724121094,0.00473756855353713,-0.046957578510046005,0.16655611991882324,-0.304395467042923,-0.05010909587144852,0.2681179344654083,-0.2715208828449249,0.07459885627031326,0.30628275871276855,-0.3882114589214325},{0.26110804080963135,0.35595691204071045,-0.2401641309261322,-0.3379272222518921,0.19959953427314758,-0.017552876845002174,0.3791067898273468,0.017909809947013855,0.09273158013820648,0.3866668939590454,0.3599458336830139,-0.0833599641919136,-0.24752366542816162,0.08529097586870193,-0.37160825729370117,0.025836804881691933,0.15763336420059204,-0.042413294315338135,0.22250235080718994,0.2866094708442688},{-0.0625518336892128,-0.03282928094267845,-0.09889167547225952,0.36184048652648926,-0.32441815733909607,0.04035908356308937,0.12951074540615082,-0.0904783383011818,-0.22094878554344177,-0.4603569209575653,-0.2843422293663025,0.3183196187019348,0.11844562739133835,-0.27497535943984985,0.0013027763925492764,-0.14555232226848602,0.4369679391384125,0.07749027013778687,-0.011215493083000183,-0.08130448311567307},{0.2574746012687683,-0.35692495107650757,0.2905582785606384,0.27390050888061523,0.2149413526058197,0.2623986601829529,-0.18238957226276398,-0.3600781559944153,0.03164262697100639,0.05808054283261299,-0.0843430832028389,0.2755308747291565,-0.0720275267958641,0.015543553046882153,-0.39168643951416016,-0.28296959400177,0.18715830147266388,0.0859115943312645,-0.40099695324897766,0.165181502699852},{0.2265925109386444,-0.23134170472621918,0.5552720427513123,0.40210357308387756,0.1879846602678299,-0.27251431345939636,-0.2008194625377655,0.07948721945285797,-2.1135423594387248e-05,0.12412594258785248,-0.4564734697341919,0.3734632134437561,0.12038273364305496,-0.2928577661514282,-0.08128345757722855,0.01519577857106924,0.09145012497901917,0.08719747513532639,-0.016868194565176964,-0.31794220209121704},{-0.22683516144752502,-0.1264123171567917,-0.018496675416827202,0.033828917890787125,-0.19458411633968353,0.408769428730011,-0.3765811324119568,0.10607825964689255,0.46762561798095703,0.07671983540058136,-0.33332163095474243,0.32761308550834656,-0.09288819134235382,0.12275583297014236,-0.10533338040113449,-0.6109475493431091,0.17370381951332092,-0.4433595538139343,-0.16469238698482513,-0.17165718972682953},{-0.42977771162986755,0.17934849858283997,0.11989007890224457,-0.1669677197933197,0.34483233094215393,0.38730230927467346,-0.1135324165225029,0.28856149315834045,0.2353973239660263,-0.040283020585775375,-0.05780496448278427,0.05011194199323654,0.10933806747198105,-0.44623294472694397,-0.2599788010120392,-0.08922799676656723,0.31586235761642456,0.41055914759635925,0.38503754138946533,0.3614586591720581},{0.0726800262928009,0.018284376710653305,0.00026727141812443733,0.38619181513786316,0.4263085722923279,0.04497529938817024,0.42667779326438904,-0.28262320160865784,0.049879040569067,0.058382079005241394,-0.2282598912715912,-0.30777987837791443,0.30072757601737976,-0.44843897223472595,-0.15431351959705353,0.05343567579984665,0.022541102021932602,0.07085994631052017,-0.1868523359298706,-0.05543529987335205},{-0.14826872944831848,-0.3043811023235321,-0.0852569043636322,0.2962736189365387,-0.3594580292701721,0.039219003170728683,-0.1333189159631729,-0.14370012283325195,0.39460107684135437,0.18902026116847992,-0.0460820235311985,-0.17218050360679626,-0.19652995467185974,0.35231372714042664,-0.26571038365364075,0.21304018795490265,-0.2315015345811844,0.20340926945209503,0.20214428007602692,0.41017377376556396},{0.06154210865497589,-0.23184743523597717,-0.014053850434720516,-0.023976948112249374,-0.2827751040458679,-0.24423359334468842,0.45188555121421814,-0.3011959493160248,-0.21379701793193817,0.010778236202895641,0.07084710896015167,-0.28670260310173035,0.051996007561683655,-0.24570976197719574,-0.374965637922287,-0.3446463644504547,0.39311379194259644,0.23538513481616974,0.3500858545303345,-0.08675574511289597},{0.40958043932914734,0.3948725759983063,0.3243808448314667,-0.13069070875644684,-0.2573866546154022,0.014310670085251331,0.3977300524711609,0.08108396828174591,0.19426800310611725,-0.2933023273944855,0.06731247156858444,-0.04051484540104866,-0.3838115632534027,0.11032203584909439,-0.23099562525749207,0.0751514732837677,-0.17269016802310944,0.29432231187820435,0.11310487985610962,-0.26778870820999146},{-0.23677760362625122,0.17911280691623688,0.2421434223651886,0.2228715419769287,0.2561347782611847,0.15944087505340576,0.47553199529647827,0.037321437150239944,0.06259404867887497,0.13304181396961212,-0.2782934308052063,0.14662617444992065,-0.12013628333806992,0.07443879544734955,-0.06720442324876785,0.2622525691986084,-0.3446043133735657,-0.2951014041900635,0.3766639828681946,-0.36796605587005615},{-0.0936112180352211,0.27467745542526245,-0.23837198317050934,0.20348377525806427,0.16726039350032806,0.1944233477115631,0.4856809675693512,-0.05130846053361893,0.31834182143211365,-0.08069873601198196,0.04924484342336655,0.11113223433494568,-0.2744911313056946,0.39019009470939636,0.02840842492878437,-0.09826783835887909,-0.29189079999923706,0.3107331395149231,0.014866854064166546,-0.13935090601444244},{0.4137471318244934,-0.19788441061973572,0.25655442476272583,-0.30523163080215454,0.10294762253761292,0.23514100909233093,-0.12249976396560669,-0.230951189994812,-0.22355066239833832,0.05869392305612564,0.18124862015247345,0.15147466957569122,0.1894664615392685,-0.07680641114711761,0.2873368263244629,-0.10520147532224655,-0.08688098192214966,-0.053985245525836945,-0.14318466186523438,0.19489459693431854},{0.24796515703201294,0.14309921860694885,0.1353156864643097,0.31024056673049927,-0.4097048044204712,0.01181862410157919,0.46924713253974915,-0.07465572655200958,0.002216927008703351,-0.013819609768688679,0.22003984451293945,-0.26107266545295715,-0.25061044096946716,0.1901988685131073,0.3183506727218628,0.33256813883781433,0.10463237762451172,-0.24343490600585938,-0.11685606092214584,0.04444525018334389},{0.24697887897491455,0.24869360029697418,0.05349936708807945,-0.21334125101566315,-0.16361676156520844,-0.2492758333683014,0.15286901593208313,0.24608059227466583,0.12043286114931107,-0.11601454019546509,0.23008248209953308,-0.0762810930609703,0.08912888914346695,-0.19986730813980103,-0.27689388394355774,0.31799209117889404,-0.28303807973861694,0.08060482889413834,-0.03811294957995415,-0.026191655546426773},{0.22048398852348328,-0.019124336540699005,-0.2090049684047699,-0.049860578030347824,0.3120514154434204,0.16205333173274994,-0.10548368096351624,-0.09913463890552521,-0.15518221259117126,0.05542352423071861,-0.09151539206504822,-0.07544906437397003,0.11112483590841293,0.29346704483032227,-0.20387022197246552,0.0938066765666008,0.24217364192008972,-0.3701818883419037,-0.25913065671920776,0.22516833245754242}};
static const float action_parameterization_distribution_linear_weight[20][4] = {{-0.1788455992937088,-0.17215639352798462,-0.3273516297340393,-0.34594422578811646},{-0.017121920362114906,-0.24268661439418793,0.20889140665531158,-0.07093403488397598},{0.10619228333234787,0.2983390688896179,0.05405094847083092,-0.15028344094753265},{0.3076260983943939,0.24367757141590118,0.30156049132347107,0.34071579575538635},{0.08451510965824127,0.11932706832885742,-0.4806063175201416,0.2380804717540741},{0.14724811911582947,0.04685046523809433,-0.023355530574917793,0.2422092705965042},{-0.474249929189682,-0.07340134680271149,-0.2434684783220291,0.3268495500087738},{0.35844385623931885,-0.17597392201423645,-0.30341440439224243,0.528678834438324},{-0.1644437313079834,0.5481453537940979,-0.3410020172595978,0.3160577714443207},{-0.11930215358734131,0.054438672959804535,-0.29147064685821533,-0.22696436941623688},{0.28539031744003296,-0.37234485149383545,-0.5402110815048218,0.22217153012752533},{0.4857659339904785,0.41942042112350464,-0.05453964322805405,-0.06085096672177315},{-0.10275770723819733,-0.4291037619113922,0.3120410442352295,0.1492471694946289},{0.26389557123184204,0.38397106528282166,-0.2860981523990631,-0.22580572962760925},{0.42398369312286377,-0.3212966322898865,-0.5224320888519287,0.11663801223039627},{0.125019371509552,-0.6316531300544739,0.20936603844165802,0.09524152427911758},{0.09723053127527237,0.2395573854446411,0.2606674134731293,-0.048155106604099274},{0.28930598497390747,-0.30685925483703613,0.327040433883667,0.2723005414009094},{-0.21025574207305908,0.03950090706348419,-0.20832814276218414,0.08944547176361084},{0.29672157764434814,0.3494894504547119,-0.4186554253101349,0.47356748580932617}};
static const float actor_encoder_self_encoder_0_bias[16] = {0.030838901177048683,0.03395852446556091,0.12456948310136795,-0.10929775983095169,0.06628552824258804,-0.09263329952955246,0.13467846810817719,0.014577796682715416,-0.061959851533174515,-0.0007024722872301936,-0.061125047504901886,-0.02517874911427498,-0.022882506251335144,0.05439086630940437,0.14976589381694794,0.0025806676130741835};
static const float actor_encoder_self_encoder_2_bias[16] = {-0.1457139402627945,0.005590063985437155,0.0006448979838751256,0.038271527737379074,-0.028972892090678215,0.06256987899541855,-0.07893875241279602,-0.0621805414557457,0.16266083717346191,0.15507057309150696,0.016522839665412903,0.017179720103740692,-0.02454625442624092,-0.0774967297911644,0.021988293156027794,-0.008369488641619682};
static const float actor_encoder_feed_forward_0_bias[20] = {-0.058285653591156006,0.0011450615711510181,-0.02878676913678646,0.006881189066916704,0.02217015065252781,0.019789453595876694,-0.08656303584575653,-0.019884277135133743,0.004461961332708597,0.001949141500517726,-0.004360932391136885,-0.02103380858898163,0.04438604414463043,-0.1234171912074089,0.04050559177994728,0.004952542018145323,0.05267605558037758,0.07794488966464996,0.008022298105061054,-0.024738749489188194};
static const float action_parameterization_distribution_linear_bias[4] = {0.017697952687740326,0.0016371156089007854,0.044690344482660294,0.022680679336190224};
static const float actor_encoder_obstacle_encoder_0_weight[16][4] = {{0.08205199986696243,-0.20989666879177094,0.4148538112640381,-0.5570303797721863},{0.1457175463438034,-0.10992685705423355,-0.28009819984436035,0.36529722809791565},{-0.28144171833992004,-0.3096392750740051,0.38355040550231934,-0.24874252080917358},{0.44088831543922424,0.18628846108913422,-0.143028125166893,-0.1772865653038025},{0.4039640724658966,0.20999489724636078,-0.17690704762935638,-0.3336559534072876},{0.2195141762495041,-0.2639009952545166,-0.28640249371528625,-0.3156295418739319},{0.36130577325820923,-0.08904898911714554,-0.45501622557640076,-0.1399918794631958},{0.3163844347000122,-0.5695297718048096,-0.5237385034561157,-0.5657618045806885},{0.14195629954338074,-0.09458275884389877,-0.6051985621452332,-0.5244479775428772},{0.4037174582481384,-0.3330911695957184,0.09745264798402786,0.20940564572811127},{-0.4637927711009979,-0.696709930896759,0.01945723593235016,0.1657816767692566},{0.07280745357275009,0.26952868700027466,-0.14286206662654877,-0.460146427154541},{0.46204933524131775,-0.06861477345228195,-0.015275216661393642,-0.08246080577373505},{-0.3625708818435669,0.19145609438419342,0.275142103433609,0.19580575823783875},{0.23810413479804993,0.16988793015480042,-0.3339304029941559,0.3956752121448517},{0.39382216334342957,-0.5515865683555603,0.1989070028066635,-0.24649310111999512}};
static const float actor_encoder_obstacle_encoder_2_weight[4][4] = {{-0.7234679460525513,-0.24410481750965118,-0.6324057579040527,0.28920209407806396},{-0.45938172936439514,0.8629584908485413,0.19301779568195343,0.5910161733627319},{-0.5049715042114258,0.2135690450668335,-0.2039898782968521,0.07211408019065857},{0.789211094379425,0.4272295832633972,0.5551309585571289,0.7027312517166138}};
static const float actor_encoder_obstacle_encoder_0_bias[4] = {0.09565795958042145,-0.1864081174135208,-0.005239469930529594,-0.09204993396997452};
static const float actor_encoder_obstacle_encoder_2_bias[4] = {0.04097268357872963,-0.09778487682342529,0.023807859048247337,-0.0649760514497757};
void networkEvaluate(struct control_t_n *control_n, const float *state_array) {
        for (int i = 0; i < self_structure[0][1]; i++) {
            output_0[i] = 0;
            for (int j = 0; j < self_structure[0][0]; j++) {
                output_0[i] += state_array[j] * actor_encoder_self_encoder_0_weight[j][i];
            }
            output_0[i] += actor_encoder_self_encoder_0_bias[i];
            output_0[i] = tanhf(output_0[i]);
        }
        
        for (int i = 0; i < self_structure[1][1]; i++) {
            output_1[i] = 0;
            for (int j = 0; j < self_structure[1][0]; j++) {
                output_1[i] += output_0[j] * actor_encoder_self_encoder_2_weight[j][i];
            }
            output_1[i] += actor_encoder_self_encoder_2_bias[i];
            output_1[i] = tanhf(output_1[i]);
        }
            
        // Concat self_embed, neighbor_embed and obst_embed
        for (int i = 0; i < self_structure[1][1]; i++) {
            output_embeds[i] = output_1[i];
        }
        for (int i = 0; i < obst_structure[1][1]; i++) {
            output_embeds[i + self_structure[1][1]] = obstacle_embeds[i];
        }
    
        // Feedforward layer
        for (int i = 0; i < self_structure[2][1]; i++) {
            output_2[i] = 0;
            for (int j = 0; j < self_structure[2][0]; j++) {
                output_2[i] += output_embeds[j] * actor_encoder_feed_forward_0_weight[j][i];
                }
            output_2[i] += actor_encoder_feed_forward_0_bias[i];
            output_2[i] = tanhf(output_2[i]);
        }
    
        for (int i = 0; i < self_structure[3][1]; i++) {
            output_3[i] = 0;
            for (int j = 0; j < self_structure[3][0]; j++) {
                output_3[i] += output_2[j] * action_parameterization_distribution_linear_weight[j][i];
            }
            output_3[i] += action_parameterization_distribution_linear_bias[i];
        }
        
        control_n->thrust_0 = output_3[0];
        control_n->thrust_1 = output_3[1];
        control_n->thrust_2 = output_3[2];
        control_n->thrust_3 = output_3[3];	
    }

void obstacleEmbedder(float obstacle_inputs[OBST_DIM]) {
        //reset embeddings accumulator to zero
        memset(obstacle_embeds, 0, sizeof(obstacle_embeds));

    
        for (int i = 0; i < obst_structure[0][1]; i++) {
            obst_output_0[i] = 0;
            for (int j = 0; j < obst_structure[0][0]; j++) {
                obst_output_0[i] += obstacle_inputs[j] * actor_encoder_obstacle_encoder_0_weight[j][i];
            }
            obst_output_0[i] += actor_encoder_obstacle_encoder_0_bias[i];
            obst_output_0[i] = tanhf(obst_output_0[i]);
        }
    
        for (int i = 0; i < obst_structure[1][1]; i++) {
            obst_output_1[i] = 0;
            for (int j = 0; j < obst_structure[1][0]; j++) {
                obst_output_1[i] += obst_output_0[j] * actor_encoder_obstacle_encoder_2_weight[j][i];
            }
            obst_output_1[i] += actor_encoder_obstacle_encoder_2_bias[i];
            obstacle_embeds[i] = tanhf(obst_output_1[i]);
        }
        }