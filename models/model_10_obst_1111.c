
#include "network_evaluate_tof.h"
#include <string.h>


#define EPS 0.000001 // 1e-6

static float obstacle_embeds[4];
static float output_embeds[20];

float base;
float exponent;
static const int self_structure [4][2] = {{18, 16},{16, 16},{20, 32},{32, 4}};
static const int obst_structure [2][2] = {{16, 4},{4, 4}};
static float output_0[16];
static float output_1[16];
static float obst_output_0[4];
static float obst_output_1[4];
static float output_2[32];
static float output_3[4];
static const float actor_encoder_self_encoder_0_weight[18][16] = {{-0.11731819808483124,0.17246568202972412,0.28857794404029846,0.13042621314525604,-0.07691898196935654,0.33977624773979187,0.025867290794849396,0.241779163479805,0.16950343549251556,-0.008387702517211437,-0.08623672276735306,0.4910205602645874,-0.029981505125761032,-0.09774080663919449,0.11210641264915466,0.15658363699913025},{0.02923804521560669,-0.04842164367437363,0.2767621576786041,0.1587192416191101,0.0897582396864891,-0.1572381556034088,-0.21724538505077362,-0.2707567512989044,0.262437105178833,0.035957541316747665,0.022003306075930595,0.0049651083536446095,-0.3112570345401764,0.07939417660236359,0.07395996898412704,0.3179457187652588},{-0.30635499954223633,-0.46601250767707825,-0.011784051544964314,0.14855976402759552,0.11773594468832016,0.2165403664112091,-0.07930509746074677,-0.1106615886092186,-0.1478356122970581,-0.6112765669822693,-0.14093522727489471,0.23630942404270172,0.008312320336699486,0.35231584310531616,0.29692426323890686,0.2504856586456299},{-0.109324149787426,-0.03615161404013634,-0.07933121919631958,0.22508558630943298,-0.0369160920381546,0.02343514747917652,0.24316655099391937,-0.17858880758285522,-0.09593376517295837,-0.08268013596534729,-0.1201833188533783,0.019889747723937035,-0.03731311112642288,-0.4049091339111328,-0.028691301122307777,-0.30978354811668396},{-0.35620859265327454,-0.08666182309389114,0.11293145269155502,0.41009774804115295,0.2032099813222885,-0.1647346168756485,-0.3002931475639343,0.18904556334018707,-0.25842756032943726,0.39488112926483154,0.09516220539808273,0.059257350862026215,0.13964517414569855,-0.405159056186676,-0.015483378432691097,0.04026192054152489},{0.22138263285160065,0.2572905123233795,-0.2874361574649811,0.08714032918214798,0.2569223642349243,-0.1679920107126236,0.16964207589626312,0.19908003509044647,-0.10216675698757172,-0.601989209651947,0.03843989595770836,0.03403706103563309,-0.4502565562725067,0.12287908792495728,0.1967240869998932,-0.001268893014639616},{-0.047136418521404266,-0.10808715969324112,0.43717285990715027,0.32036662101745605,-0.09810838848352432,0.6662360429763794,-0.26534417271614075,0.12412494421005249,0.15108643472194672,0.3970133364200592,-0.46382081508636475,0.2676412761211395,-0.011503058485686779,0.44746652245521545,-0.12556833028793335,0.08606746792793274},{-0.06743369996547699,0.3834623396396637,-0.036801304668188095,0.009116414934396744,-0.12250354140996933,0.4778698980808258,-0.11196089535951614,-0.509921133518219,0.03256760537624359,0.29715827107429504,-0.14503946900367737,0.11350107938051224,0.19939854741096497,-0.14189404249191284,0.3672183156013489,-0.04429222270846367},{-0.5230720043182373,0.3552309572696686,-0.016565455123782158,-0.18845723569393158,-0.2049856036901474,-0.06177430599927902,0.2899336516857147,-0.30308327078819275,0.32967084646224976,0.3295000195503235,-0.8942097425460815,0.3180183172225952,-0.17321714758872986,-0.4274038076400757,-0.2933111786842346,0.1381550133228302},{-0.0634227991104126,0.1780187338590622,-0.22435732185840607,-0.3194379210472107,-0.09226834774017334,-0.45124056935310364,-0.2712927758693695,0.5100778341293335,0.13181039690971375,0.44819357991218567,0.14257386326789856,0.4136919379234314,-0.11375933885574341,-0.36975595355033875,0.18781708180904388,0.03187455236911774},{0.03810766339302063,-0.3213772475719452,0.35833337903022766,-0.059953831136226654,0.15659353137016296,0.21502213180065155,-0.10202877968549728,0.3407520353794098,-0.15185487270355225,-0.03527778387069702,-0.07327498495578766,-0.2808740735054016,0.10832250118255615,-0.3033057153224945,0.3362148404121399,0.012345490045845509},{-0.3490634262561798,0.18112586438655853,-0.04140131548047066,0.39506661891937256,0.6902444362640381,0.3485671281814575,-0.39856159687042236,0.06397410482168198,0.40424153208732605,0.5122895240783691,0.24792876839637756,-0.05115930736064911,0.2805565893650055,-0.22692228853702545,-0.07035364210605621,0.35560905933380127},{-0.25522735714912415,-0.10982011258602142,0.10251444578170776,-0.07633822411298752,-0.5363675355911255,-0.27485930919647217,0.637965738773346,-0.023048365488648415,-0.2180173546075821,0.26674503087997437,-0.4978548586368561,0.3045724630355835,-0.023582613095641136,-0.15435738861560822,0.7055668830871582,-0.31630343198776245},{-0.31630662083625793,0.5305474400520325,0.5105026364326477,-0.013239050284028053,0.6806710362434387,0.41664958000183105,-0.41497835516929626,-0.4859323799610138,0.22097031772136688,-0.04998192563652992,-0.3629920780658722,0.34895530343055725,-0.515380859375,-0.26719242334365845,-0.46173155307769775,0.29559171199798584},{0.12995868921279907,0.1136825755238533,-0.3679881691932678,-0.432792603969574,0.29198604822158813,-0.2405594438314438,0.462523877620697,0.30946290493011475,-0.07886902242898941,0.12987737357616425,0.1912676841020584,-0.09704805910587311,0.17811354994773865,0.1372353583574295,-0.2773236930370331,-0.007194148376584053},{-0.22586052119731903,0.25831952691078186,-0.2516518235206604,0.37620455026626587,0.16025882959365845,-0.055028095841407776,0.07691014558076859,-0.0906653180718422,-0.10227005928754807,0.36408573389053345,-0.0496390201151371,0.02281527779996395,-0.07610411196947098,0.12934498488903046,-0.09183066338300705,0.11092998087406158},{0.06736046820878983,-0.015716252848505974,0.18044133484363556,0.02967432141304016,0.003278841497376561,0.30600833892822266,-0.34404316544532776,0.24459286034107208,0.3574877977371216,-0.23520280420780182,0.09454423934221268,0.19275659322738647,0.0016798041760921478,-0.09756754338741302,-0.1979445368051529,0.1215209811925888},{0.27011001110076904,0.40459761023521423,0.41347843408584595,0.25373575091362,0.18448512256145477,0.2722545862197876,0.25696468353271484,-0.38182270526885986,0.2450852245092392,-0.24374046921730042,-0.1354786902666092,-0.2782076895236969,0.07503163814544678,0.13124829530715942,-0.013249747455120087,-0.10803737491369247}};
static const float actor_encoder_self_encoder_2_weight[16][16] = {{0.2071583867073059,0.02621961385011673,-0.47359928488731384,-0.07629220932722092,0.03772101551294327,0.3022361397743225,-0.34445226192474365,0.4267632067203522,0.3321673274040222,0.12456629425287247,0.46674293279647827,-0.16185300052165985,0.19320335984230042,0.22645767033100128,-0.14250482618808746,0.2678888738155365},{-0.46904388070106506,-0.39637142419815063,0.20519712567329407,-0.17205236852169037,0.3292085826396942,-0.3441521227359772,0.34208276867866516,-0.08633202314376831,0.26118743419647217,-0.03520430997014046,0.1988239735364914,0.3985162377357483,-0.0570022314786911,0.2391328513622284,0.31781843304634094,0.13961951434612274},{-0.0816258117556572,-0.3752162456512451,-0.5299476981163025,-0.47075986862182617,0.4105256199836731,-0.3573349714279175,0.020855549722909927,-0.3271982669830322,-0.3721822500228882,-0.06364250928163528,0.11390753835439682,0.06184148043394089,-0.38178128004074097,-0.37363165616989136,0.3891812264919281,0.019788043573498726},{-0.1511542648077011,0.23006080090999603,0.17114771902561188,0.40154096484184265,-0.005926464684307575,-0.2904764413833618,0.24527738988399506,-0.06809210777282715,-0.27612730860710144,0.23542451858520508,-0.18946704268455505,0.021430805325508118,-0.5017070770263672,-0.2717946171760559,0.3426707684993744,0.4007878601551056},{0.10622410476207733,-0.44126057624816895,-0.16856727004051208,0.2098323553800583,-0.23156248033046722,-0.015423561446368694,0.5194491147994995,-0.22026927769184113,0.2457340806722641,-0.32027143239974976,-0.33055633306503296,-0.18505825102329254,0.5134824514389038,-0.06934379041194916,0.026943542063236237,-0.14884690940380096},{-0.057327933609485626,0.01582249626517296,-0.3952217102050781,-0.18724550306797028,0.13056616485118866,0.01716693304479122,0.23498034477233887,0.2665826976299286,-0.15268030762672424,-0.2636067867279053,-0.029425237327814102,-0.11731615662574768,0.0826599970459938,0.10524557530879974,0.2890254855155945,0.4568190276622772},{0.12140245735645294,0.089695505797863,-0.059096809476614,-0.22418777644634247,0.1860620379447937,-0.32874253392219543,-0.3798307478427887,-0.33881211280822754,0.14566166698932648,-0.13754861056804657,0.039032790809869766,0.23997467756271362,0.12409564852714539,0.4562009572982788,-0.22473770380020142,-0.12319374829530716},{0.004172989167273045,0.44025930762290955,-0.2916092872619629,0.04513886943459511,-0.3493061065673828,0.2548244893550873,0.1922616958618164,0.16005973517894745,-0.35686418414115906,0.40982064604759216,0.22371473908424377,-0.0427868478000164,0.3266902565956116,-0.0927971675992012,0.2732769548892975,-0.0011881851824000478},{-0.12284588068723679,-0.1352820098400116,-0.2814045250415802,-0.2956315875053406,0.183014377951622,-0.11369217932224274,0.04534759372472763,0.366690069437027,-0.2381543070077896,-0.3630302846431732,0.21871212124824524,0.16203872859477997,0.026925059035420418,0.20808462798595428,0.2499106079339981,-0.20804564654827118},{0.2672407627105713,0.013184919022023678,-0.267716646194458,-0.19016030430793762,0.40598365664482117,0.25199398398399353,-0.177692249417305,0.3413460850715637,0.2809681296348572,-0.46575433015823364,0.10729630291461945,0.15776047110557556,-0.31810736656188965,0.04068810120224953,0.32013726234436035,-0.15890854597091675},{-0.34494590759277344,-0.4733007550239563,-0.025332745164632797,-0.052901316434144974,0.17337088286876678,0.3010118305683136,0.21921910345554352,0.42996981739997864,-0.0503072589635849,0.20887185633182526,0.21446575224399567,-0.3499689996242523,0.3358714282512665,0.2381642907857895,-0.2155769020318985,-0.07704474031925201},{-0.26004165410995483,0.29033371806144714,0.08746907860040665,0.3782966732978821,-0.08456269651651382,0.16678427159786224,-0.5114589929580688,-0.09885106980800629,0.45116564631462097,-0.10322202742099762,-0.26992860436439514,-0.09544525295495987,0.2275891751050949,-0.07237580418586731,-0.20320984721183777,-0.18067404627799988},{-0.03617192059755325,-0.3566761016845703,0.3045880198478699,-0.3806295394897461,0.1515020877122879,-0.20438237488269806,-0.026923097670078278,-0.1540296971797943,-0.14819572865962982,0.26550304889678955,0.25812509655952454,-0.036207061260938644,-0.2790948748588562,0.14691060781478882,0.1317276805639267,0.023242488503456116},{-0.28678447008132935,-0.2934955954551697,0.4174426794052124,-0.0016811274690553546,0.37468039989471436,0.21813371777534485,-0.07255697250366211,0.05233657360076904,-0.48012620210647583,0.34064894914627075,0.0027885735034942627,-0.20604459941387177,-0.26407065987586975,-0.18122640252113342,-0.3536354899406433,0.3231542706489563},{-0.026673318818211555,0.1556328684091568,-0.10002952069044113,0.19253982603549957,-0.20302096009254456,0.09166459739208221,0.015948744490742683,-0.16476117074489594,-0.39385974407196045,0.044504497200250626,0.2607046365737915,0.3178933262825012,-0.25654202699661255,0.19737601280212402,0.07557877898216248,0.5386439561843872},{-0.28712400794029236,0.057853829115629196,-0.0360710434615612,-0.06903275102376938,-0.23789045214653015,0.343178391456604,0.47479620575904846,-0.23580873012542725,-0.10420020669698715,-0.15530996024608612,-0.40220901370048523,-0.35813021659851074,0.02883879840373993,-0.23127765953540802,-0.23428407311439514,-0.07915524393320084}};
static const float actor_encoder_feed_forward_0_weight[20][32] = {{0.08107917010784149,-0.16480965912342072,0.21107786893844604,0.02920377440750599,-0.12237069010734558,0.09593230485916138,-0.09596642851829529,-0.11852708458900452,-0.16650958359241486,0.26163721084594727,0.19193387031555176,-0.004506309051066637,0.07721510529518127,0.04671456664800644,0.1718296855688095,-0.09562386572360992,-0.2560075521469116,-0.17476829886436462,-0.2321760207414627,-0.2266816943883896,0.3147646486759186,0.12376611679792404,-0.1385207623243332,-0.14475663006305695,-0.25774940848350525,-0.047305136919021606,-0.19444270431995392,-0.13034527003765106,0.15915806591510773,0.04081270843744278,-0.009591979905962944,-0.11752571910619736},{-0.1860356330871582,0.12843315303325653,0.07552933692932129,-0.1140948161482811,0.15451361238956451,-0.27322447299957275,-0.4427112340927124,0.011236399412155151,-0.1799023300409317,0.22392120957374573,0.2982078790664673,-0.2890034317970276,0.3035435080528259,-0.24304887652397156,-0.0936674028635025,-0.24497820436954498,0.20446893572807312,-0.22468777000904083,-0.02418396808207035,-0.23050342500209808,0.2570505440235138,-0.25175514817237854,-0.32231613993644714,-0.10262027382850647,-0.18062590062618256,0.3287467062473297,0.23822902143001556,-0.43579337000846863,0.27234184741973877,0.20777569711208344,0.15408313274383545,-0.2131366729736328},{-0.011942319571971893,0.1546289026737213,0.19325047731399536,-0.13477569818496704,-0.05903506651520729,0.30245423316955566,-0.057369913905858994,0.1516253650188446,0.29644420742988586,0.015569430775940418,0.11896471679210663,-0.05435855686664581,-0.014652671292424202,-0.3654291331768036,-0.3043898046016693,-0.05461392551660538,-0.00015822492423467338,0.07968081533908844,0.21478448808193207,-0.04813385754823685,-0.2577425241470337,0.2565852403640747,-0.07876091450452805,0.15890264511108398,0.08631878346204758,-0.2669583559036255,-0.1266636848449707,0.2801325023174286,-0.26726943254470825,-0.3516963720321655,-0.2654065191745758,0.03185382857918739},{0.25015538930892944,-0.2278868705034256,0.08339934051036835,-0.1029433086514473,-0.036188557744026184,-0.4499009847640991,-0.3450552523136139,0.17635215818881989,0.3648889660835266,0.1744733452796936,0.0919405072927475,0.2625276744365692,0.3016778230667114,0.05192180350422859,0.23470525443553925,0.07771169394254684,0.25030532479286194,0.23044829070568085,0.16858771443367004,-0.03687914460897446,-0.09712419658899307,0.02959524467587471,-0.12490478157997131,0.16259919106960297,-0.10851593315601349,-0.1617899239063263,0.34415608644485474,-0.22379833459854126,-0.167560875415802,-0.13530181348323822,-0.17480269074440002,0.0973307341337204},{0.31945928931236267,-0.18022693693637848,-0.037456266582012177,-0.28305813670158386,-0.07188167423009872,-0.024674655869603157,0.1271704137325287,-0.3392362892627716,-0.05971220135688782,-0.13460665941238403,-0.06645664572715759,-0.3462732136249542,-0.1287117302417755,-0.1485070288181305,-0.05899745598435402,0.02372649312019348,-0.34537172317504883,0.038252800703048706,0.15534576773643494,-0.3065587282180786,0.13251248002052307,-0.015717122703790665,-0.0321950800716877,0.0448383130133152,-0.2099587619304657,0.0269861351698637,-0.43559807538986206,0.025088272988796234,-0.01635037735104561,0.19644732773303986,-0.27685481309890747,-0.04131053760647774},{0.02447781339287758,-0.11238718777894974,0.22926390171051025,0.22304777801036835,0.3036116063594818,-0.19847336411476135,-0.19490784406661987,-0.2242816686630249,-0.05595564469695091,-0.035316381603479385,-0.003315908368676901,0.00580731313675642,0.19681501388549805,-0.16817624866962433,-0.09340130537748337,-0.14832283556461334,0.06831394135951996,0.17220373451709747,0.3945373594760895,-0.07770401239395142,-0.039443306624889374,-0.387084037065506,-0.03756212815642357,-0.0874677449464798,-0.1897284984588623,0.003906731493771076,0.19662326574325562,0.30518922209739685,-0.20469075441360474,-0.13181860744953156,0.23101432621479034,0.11992385238409042},{0.17602719366550446,-0.38348308205604553,-0.25652584433555603,0.0845239907503128,0.1580994725227356,-0.2091343104839325,0.24596652388572693,0.25366976857185364,0.22647157311439514,-0.21502302587032318,-0.37881985306739807,0.04137757420539856,0.06291811913251877,0.15896093845367432,-0.28422483801841736,-0.23168417811393738,-0.04801075533032417,0.317384272813797,-0.3439054489135742,-0.11177632212638855,-0.0699128732085228,-0.19090627133846283,0.24174505472183228,0.08049511909484863,0.04056864604353905,0.13093270361423492,0.16834944486618042,0.1760179102420807,-0.017707493156194687,-0.04120459035038948,0.17228713631629944,-0.20592306554317474},{0.012364459224045277,0.2058243602514267,0.3179057836532593,0.3076808750629425,-0.051221758127212524,0.09186363220214844,0.4247361123561859,0.05826688930392265,0.10564549267292023,-0.2709006369113922,0.3011102080345154,-0.2009599208831787,-0.22728314995765686,-0.16051506996154785,0.3662208318710327,0.1267739087343216,-0.29007601737976074,-0.0263645201921463,0.16939792037010193,-0.16246391832828522,0.2588212490081787,-0.3587065041065216,0.13792301714420319,-0.09147553890943527,-0.12186618894338608,-0.12113253772258759,-0.12993115186691284,0.09990857541561127,-0.05207907035946846,0.27789896726608276,-0.30499809980392456,-0.1807078719139099},{0.01475975289940834,0.34558963775634766,0.2271386981010437,-0.1652727872133255,0.15072883665561676,0.18481110036373138,0.11225150525569916,-0.09508441388607025,0.06191347911953926,0.048669569194316864,0.19954416155815125,-0.08285248279571533,-0.37357836961746216,0.03995135799050331,0.011590123176574707,-0.07309996336698532,-0.22616258263587952,-0.04437443986535072,0.23131509125232697,-0.3935452699661255,-0.09492548555135727,0.14818337559700012,-0.05727457255125046,0.18094804883003235,-0.3041713237762451,-0.09787550568580627,-0.09521115571260452,0.10424131155014038,-0.15468177199363708,-0.3391249477863312,-0.19622936844825745,0.10696249455213547},{-0.3188501298427582,-0.16342440247535706,-0.013394007459282875,0.07359747588634491,-0.22162383794784546,0.024965522810816765,-0.2412571907043457,-0.29498186707496643,0.2672485113143921,0.10579513758420944,0.1729503870010376,-0.1734074205160141,-0.2056974619626999,-0.1555807739496231,0.254271924495697,-0.2812366485595703,0.2823958098888397,0.03150027617812157,0.029342254623770714,0.028352292254567146,0.11672420054674149,-0.19140954315662384,0.008326031267642975,-0.1855667680501938,0.32508838176727295,0.2186119258403778,0.08325617015361786,0.30304208397865295,0.1605985462665558,0.26588499546051025,0.21187566220760345,-0.3670387864112854},{-0.08337878435850143,0.2451600879430771,0.14367899298667908,0.17359113693237305,0.004643368534743786,0.2833382189273834,0.13653473556041718,-0.18428988754749298,0.006978195160627365,0.09615165740251541,0.0522928424179554,0.10154017806053162,-0.12039171159267426,0.14320015907287598,0.2821219563484192,0.023774784058332443,0.10419968515634537,-0.35431262850761414,0.30268988013267517,0.022970210760831833,-0.10573803633451462,0.1671333909034729,0.04772874712944031,0.16748499870300293,0.24883444607257843,-0.4857838451862335,0.07024902105331421,0.2777193784713745,-0.11973577737808228,-0.12191352993249893,-0.1722525805234909,0.06296933442354202},{0.0505426861345768,0.04247771203517914,0.19570975005626678,-0.06613367050886154,0.009734900668263435,-0.2898791432380676,-0.2767123878002167,0.13283419609069824,-0.14507904648780823,-0.046218425035476685,0.2505263090133667,0.26785367727279663,0.05897480994462967,0.10266699641942978,0.0413227379322052,-0.2625252604484558,0.1289898157119751,-0.11618436127901077,0.18613025546073914,0.18937534093856812,-0.06064325571060181,0.03702612593770027,-0.3967999815940857,0.3264531195163727,0.012203308753669262,0.08376438170671463,0.3757154941558838,-0.08913309127092361,0.3011941611766815,-0.3435536026954651,0.0010686772875487804,0.37659862637519836},{-0.26302945613861084,-0.2603481709957123,0.05518858879804611,0.02433387003839016,0.08791846036911011,0.2607056796550751,-0.09847527742385864,-0.31828415393829346,0.2722717821598053,-0.32971227169036865,0.10785304009914398,0.012395111843943596,0.10871176421642303,-0.08635366708040237,-0.17547839879989624,0.126628577709198,0.07035937905311584,0.12308146804571152,0.057027965784072876,0.02237897366285324,0.11853364109992981,0.29748016595840454,-0.036735616624355316,0.31597864627838135,0.04712572321295738,0.07346709817647934,0.23770739138126373,0.3131214678287506,0.2590332627296448,-0.1267266571521759,-0.34591415524482727,0.09597042202949524},{0.03383099287748337,-0.20829202234745026,0.20614135265350342,-0.012062358669936657,-0.22922588884830475,0.26808595657348633,0.2203092873096466,0.03198724612593651,0.11987506598234177,0.12449944019317627,-0.196334570646286,0.0632067620754242,-0.2384602427482605,0.14756615459918976,0.3286730647087097,-0.25159376859664917,-0.19660267233848572,-0.29877519607543945,-0.1816612184047699,-0.1167820543050766,-0.39546969532966614,-0.16974952816963196,-0.03889487683773041,0.13038384914398193,0.09928550571203232,-0.27447178959846497,0.13365042209625244,-0.008955854922533035,-0.3432280719280243,0.19735167920589447,0.14121121168136597,-0.07409545034170151},{0.0554979033768177,0.3379887044429779,0.24502643942832947,0.14511802792549133,-0.08607737720012665,-0.309869647026062,0.2076425701379776,0.13472582399845123,0.030874233692884445,-0.11859746277332306,-0.1545925736427307,0.12702882289886475,0.10362403839826584,0.20467302203178406,-0.11332327872514725,0.14557582139968872,-0.04606994614005089,-0.07920824736356735,-0.3635202944278717,0.046861592680215836,-0.2688659727573395,0.28213751316070557,-0.01754719391465187,0.091376394033432,0.005989121738821268,0.039245396852493286,0.11316955834627151,-0.1170768141746521,0.21607083082199097,0.4095214903354645,0.21324118971824646,-0.17626814544200897},{-0.1340837925672531,0.23449432849884033,-0.2990051209926605,0.14773297309875488,0.18232358992099762,0.15673543512821198,-0.26509955525398254,-0.14364269375801086,-0.2252512127161026,0.1350538283586502,0.0326099656522274,-0.07015343010425568,0.21989305317401886,-0.08609941601753235,0.34145206212997437,-0.31846943497657776,-0.14359654486179352,0.3515184223651886,0.009029527194797993,0.3125784695148468,0.16701391339302063,0.3906501531600952,-0.213709756731987,-0.11281508952379227,-0.21139198541641235,0.2886752486228943,-0.04778086394071579,0.11828560382127762,-0.00881690438836813,-0.12022051215171814,0.22195596992969513,-0.09247955679893494},{0.06525120884180069,-0.10581299662590027,0.3360447883605957,-0.27056849002838135,-0.2721100449562073,0.3681747615337372,0.2880116403102875,-0.07739899307489395,-0.13860563933849335,-0.15150119364261627,0.06308779865503311,-0.22485984861850739,-0.27580615878105164,0.05616435036063194,0.22662213444709778,0.20982934534549713,0.2586307227611542,0.3100036084651947,0.3074231743812561,-0.0023955805227160454,0.19973747432231903,-0.11130520701408386,0.07050104439258575,0.18905387818813324,0.17164760828018188,0.006596085149794817,-0.18990306556224823,0.06300252676010132,-0.04860030487179756,-0.19712354242801666,-0.3494178354740143,-0.17526952922344208},{-0.2766325771808624,-0.021833905950188637,-0.13629193603992462,0.3048146665096283,0.3665603995323181,-0.001058085705153644,-0.05716339498758316,0.19311165809631348,0.07024781405925751,-0.011386317200958729,0.08019156754016876,-0.23588937520980835,0.1203976422548294,0.06256512552499771,0.11854735016822815,-0.2925543189048767,0.06394308805465698,0.2820734977722168,0.017247764393687248,-0.2628401219844818,-0.0902399867773056,0.0006653151358477771,-0.04708283022046089,-0.3029099404811859,0.21905666589736938,-0.009788681752979755,0.08184187859296799,0.2196209877729416,0.12133384495973587,-0.022765494883060455,0.3412935733795166,0.2646285891532898},{-0.04962477833032608,0.19781522452831268,-0.2592271566390991,0.29250481724739075,-0.02172662504017353,0.011471071280539036,0.12197533249855042,-0.1397329568862915,0.2681306302547455,0.36594292521476746,0.2507264316082001,0.1960429847240448,-0.2286735326051712,-0.21884089708328247,0.04594426229596138,0.06674704700708389,0.17960751056671143,-0.27486833930015564,0.32796409726142883,-0.1524641513824463,0.3156702220439911,-0.002252627396956086,-0.017937054857611656,0.11967665702104568,-0.10285282880067825,0.11013778299093246,0.20694993436336517,-0.14700016379356384,0.08434101939201355,0.1025519147515297,-0.32441064715385437,0.19949971139431},{-0.172512486577034,0.2267766147851944,0.18187831342220306,-0.14144426584243774,0.21236002445220947,0.03985447809100151,-0.31618842482566833,0.22461865842342377,-0.2547789216041565,-0.19569748640060425,0.009609995409846306,0.09195967763662338,0.04801683872938156,0.3059733510017395,0.24730798602104187,0.03728242591023445,-0.2593735158443451,-0.23847348988056183,-0.1846213936805725,0.3097245395183563,0.15097393095493317,-0.24697867035865784,-0.4152054488658905,-0.26087483763694763,-0.2761792838573456,0.09897764027118683,-0.11257518082857132,-0.07966072857379913,0.1775577962398529,0.03641607239842415,0.11776341497898102,-0.08987884968519211}};
static const float action_parameterization_distribution_linear_weight[32][4] = {{0.10441608726978302,-0.12133988738059998,-0.03054128773510456,-0.11247337609529495},{-0.005550654139369726,0.2721634805202484,0.24814237654209137,-0.06458844989538193},{0.32919397950172424,-0.3181114196777344,0.12692594528198242,0.1177576407790184},{0.3205794394016266,0.08048061281442642,-0.04691396281123161,0.2471705973148346},{0.3200281858444214,0.29154592752456665,-0.16233877837657928,0.31239473819732666},{-0.2434452921152115,0.12370048463344574,0.4775676429271698,0.1528785228729248},{0.4342925250530243,0.22728650271892548,0.04789169505238533,0.27699509263038635},{-0.10620485246181488,-0.3112582862377167,-0.18827924132347107,-0.46212300658226013},{-0.031182080507278442,-0.3373161256313324,-0.4623335301876068,0.33968228101730347},{0.09874966740608215,-0.30567196011543274,-0.3897985517978668,-0.011530637741088867},{0.20790167152881622,0.264242023229599,0.12382606416940689,0.1395513415336609},{-0.23043586313724518,-0.032784346491098404,0.09622553735971451,-0.12592411041259766},{-0.09053763002157211,-0.09347566962242126,-0.4025115966796875,0.2420927733182907},{0.35401955246925354,0.030540456995368004,-0.06407405436038971,-0.07268182933330536},{0.014321615919470787,-0.2634826898574829,0.3909897208213806,-0.13194121420383453},{0.3667573928833008,0.3022063374519348,-0.10008501261472702,0.23920002579689026},{-0.4009227752685547,0.08592663705348969,-0.13634386658668518,-0.08685408532619476},{0.08203853666782379,-0.40711554884910583,-0.20540399849414825,-0.35006096959114075},{-0.3325665295124054,-0.20079077780246735,-0.1981094628572464,0.4704893231391907},{-0.3058313727378845,-0.45296967029571533,-0.009472887963056564,0.24547326564788818},{-0.24139118194580078,-0.1204664334654808,-0.03258994221687317,0.378129243850708},{-0.2456166297197342,-0.36038121581077576,-0.013790426775813103,-0.21741734445095062},{-0.03381957858800888,-0.3631439805030823,-0.05463077872991562,0.09973768889904022},{0.45806118845939636,0.013191231526434422,-0.3172343075275421,-0.04392355680465698},{-0.42238956689834595,-0.07568766921758652,-0.004181068856269121,-0.3119218051433563},{-0.14083662629127502,-0.3312261998653412,-0.2040875107049942,-0.22284278273582458},{-0.2981303036212921,0.237142875790596,0.08651052415370941,-0.12979288399219513},{0.16481901705265045,-0.29719817638397217,-0.29791855812072754,0.4328276813030243},{0.11479391902685165,0.19494366645812988,-0.100418321788311,0.1106598973274231},{0.0654989629983902,-0.28838375210762024,0.38008520007133484,-0.055212005972862244},{0.15022887289524078,0.05609949678182602,-0.09865765273571014,0.34900611639022827},{0.262484073638916,0.3275102376937866,-0.06873185932636261,-0.15367795526981354}};
static const float actor_encoder_self_encoder_0_bias[16] = {-0.10102564096450806,-0.049242015928030014,-0.06312940269708633,0.02578922174870968,0.15417379140853882,-0.2593758702278137,0.03781872242689133,0.11124960333108902,-0.06929179280996323,-0.10795658081769943,-0.04012901335954666,0.10909149795770645,-0.017644215375185013,-0.02901564911007881,0.0002113538794219494,-0.0025402791798114777};
static const float actor_encoder_self_encoder_2_bias[16] = {-0.20785710215568542,-0.029030712321400642,0.06008018180727959,-0.054481636732816696,-0.04839169606566429,-0.12663765251636505,0.00710305105894804,-0.0016228512395173311,-0.004432732705026865,-0.044421590864658356,0.011195305734872818,-0.06488115340471268,0.057409148663282394,0.04709135368466377,-0.05732903257012367,-0.11149990558624268};
static const float actor_encoder_feed_forward_0_bias[32] = {-0.04202716052532196,0.04175568372011185,0.027038319036364555,0.05886507406830788,-1.5388117390102707e-05,0.13248096406459808,0.10492440313100815,-0.038410793989896774,-0.005613240879029036,-0.07396125793457031,0.05319192260503769,0.019656583666801453,-0.09129968285560608,-0.011302286759018898,-0.001059808419086039,0.01676994562149048,0.03494536131620407,-0.016346048563718796,-0.030978310853242874,-0.0737958699464798,-0.03571367636322975,-0.026295587420463562,0.04257594048976898,-0.03419375419616699,0.021697234362363815,-0.04107162728905678,0.08506769686937332,-0.014306834898889065,-0.0572117418050766,0.016526857390999794,-0.05236414819955826,0.06533558666706085};
static const float action_parameterization_distribution_linear_bias[4] = {-0.0007603886188007891,0.026698244735598564,0.05783619359135628,0.009004834108054638};
static const float actor_encoder_obstacle_encoder_0_weight[16][4] = {{-0.027910636737942696,0.5885629057884216,-0.28858867287635803,0.262775719165802},{-0.2377960979938507,-0.3697989284992218,-0.12904705107212067,0.24556535482406616},{-0.11588070541620255,0.7031233906745911,0.058407723903656006,-0.23349858820438385},{0.51725834608078,-0.11177686601877213,0.7160436511039734,-0.11450129747390747},{0.10861667990684509,-0.19989775121212006,0.532041609287262,0.15497922897338867},{-0.2864122688770294,0.2337988018989563,-0.2554570436477661,0.024249130859971046},{-0.3612111210823059,-0.1622018963098526,0.2946273982524872,0.3237260580062866},{0.509254515171051,0.6073383092880249,0.472576767206192,-0.5454617738723755},{-0.05315515026450157,-0.08029692620038986,0.18725134432315826,-0.4764094352722168},{0.39124971628189087,0.4235928952693939,0.2028583437204361,0.21002963185310364},{0.5844619274139404,0.11846452206373215,-0.4379843473434448,-0.33526554703712463},{-0.2236648052930832,0.18160736560821533,0.12864920496940613,-0.30928683280944824},{0.6334330439567566,-0.3456938862800598,-0.019247429445385933,-0.2891310453414917},{-0.26259249448776245,0.4566691815853119,-0.31971800327301025,-0.2701241075992584},{0.1514713615179062,0.3109818398952484,-0.051331039518117905,0.28655391931533813},{0.40194693207740784,0.4084038734436035,0.029922662302851677,0.29130488634109497}};
static const float actor_encoder_obstacle_encoder_2_weight[4][4] = {{0.8705115914344788,0.09314996749162674,-0.32104480266571045,-0.4107683300971985},{0.09532223641872406,-0.2835419178009033,0.06571564823389053,0.43437907099723816},{-0.6854934096336365,0.22750262916088104,0.7781803011894226,-0.5951377153396606},{-0.37490108609199524,0.2464628368616104,-0.4567594826221466,0.8081861734390259}};
static const float actor_encoder_obstacle_encoder_0_bias[4] = {0.13853691518306732,0.011558199301362038,0.0641772598028183,-0.015040737576782703};
static const float actor_encoder_obstacle_encoder_2_bias[4] = {0.07646757364273071,0.12019769102334976,-0.08118276298046112,-0.0428861565887928};
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
