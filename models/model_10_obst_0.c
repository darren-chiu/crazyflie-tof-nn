
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
static const float actor_encoder_self_encoder_0_weight[18][16] = {{0.1408904790878296,-0.22327683866024017,-0.05471765249967575,-0.015868820250034332,-0.03533821925520897,-0.040262963622808456,-0.23765070736408234,0.017319899052381516,0.11227846890687943,0.0373210534453392,-0.3860189914703369,-0.19177861511707306,0.14250819385051727,-0.5546680092811584,0.25714072585105896,-0.01353226788341999},{0.22663430869579315,0.1986808329820633,0.013870510272681713,-0.042543280869722366,0.09039520472288132,0.2164119929075241,0.12602148950099945,-0.09648433327674866,0.23426735401153564,-0.012936266139149666,-0.07231499254703522,-0.14661051332950592,-0.0999414324760437,-0.4280886650085449,0.38555005192756653,0.01837172918021679},{0.3715120851993561,0.23409466445446014,0.19576510787010193,-0.3932237923145294,-0.05123989284038544,-0.43129241466522217,-0.16283933818340302,0.4788244068622589,0.018026167526841164,-0.6181502938270569,-0.011415421962738037,0.1881016194820404,-0.08650711923837662,0.07440967112779617,0.17367438971996307,-0.19146159291267395},{-0.36450907588005066,-0.013563611544668674,0.4076603055000305,0.2939547002315521,-0.12645059823989868,-0.054146263748407364,0.05364363640546799,-0.025215033441781998,0.059638142585754395,-0.052401814609766006,-0.47527024149894714,-0.3041352927684784,-0.1987553983926773,0.04333491995930672,-0.27850061655044556,0.12143789231777191},{-0.01207444816827774,0.06719037890434265,-0.031538885086774826,-0.28607675433158875,-0.18740470707416534,-0.12561963498592377,0.06998128443956375,0.26174625754356384,0.054199814796447754,0.3749168813228607,-0.21978606283664703,0.06614833325147629,-0.26467365026474,-0.4190389811992645,-0.03526722639799118,0.3254580497741699},{-0.08184218406677246,0.14037300646305084,0.2970830798149109,0.0036044546868652105,0.11263685673475266,-0.45910510420799255,-0.020675400272011757,-0.2221878170967102,0.5417600870132446,-0.1092618778347969,0.03366629034280777,0.1183217242360115,0.20631039142608643,0.17333360016345978,0.1613607257604599,-0.0662841945886612},{-0.2016846388578415,-0.1944236010313034,-0.14118656516075134,-0.1875649243593216,0.22907263040542603,0.3245965838432312,0.46617332100868225,-0.013827153481543064,-0.21081264317035675,0.006655355449765921,-0.1675742119550705,-0.1161334440112114,-0.26344823837280273,0.21235819160938263,0.09894713759422302,-0.35952499508857727},{0.11628055572509766,0.1930212825536728,-0.06103023514151573,0.2645164728164673,-0.33806005120277405,0.39374107122421265,-0.18849802017211914,0.1143634170293808,0.29708945751190186,-0.3037283718585968,0.4155650734901428,0.17713043093681335,0.1998840570449829,0.12735390663146973,0.4582682251930237,-0.3312985897064209},{-0.21644937992095947,-0.7139650583267212,0.47285887598991394,0.19016830623149872,-0.4087836444377899,-0.26666539907455444,0.0687108188867569,0.15813001990318298,-0.22642450034618378,-0.005638742353767157,0.1269516497850418,-0.7007570862770081,0.028391554951667786,0.11995232850313187,0.28106212615966797,0.3086663782596588},{0.22621574997901917,0.32806649804115295,-0.47079142928123474,0.17481808364391327,0.10090175271034241,0.12161415070295334,0.2601180672645569,0.2771247625350952,-0.059133607894182205,0.05446342006325722,-0.4387626349925995,0.2015281617641449,0.27437713742256165,-0.07185263931751251,0.15514902770519257,-0.03322577476501465},{0.26968395709991455,-0.13355091214179993,0.18092268705368042,-0.1951376497745514,0.28431445360183716,0.20809052884578705,0.10933800041675568,-0.055103596299886703,-0.4233783185482025,0.44891291856765747,-0.4595482647418976,0.13352954387664795,0.4841713309288025,-0.11130201071500778,-0.08877749741077423,0.3153381943702698},{0.18678180873394012,-0.33931964635849,0.45442089438438416,-0.28539377450942993,0.1475335955619812,0.19666947424411774,-0.2558358311653137,-0.34093695878982544,0.3772210478782654,0.4196228086948395,0.4452126622200012,0.1287945657968521,-0.4027252495288849,-0.2497161328792572,0.3972012996673584,0.5591005086898804},{0.24324482679367065,-0.35770437121391296,-0.05188214033842087,0.17873790860176086,0.11159523576498032,0.16115398705005646,0.24530117213726044,0.3551742136478424,-0.3487405478954315,0.03730214387178421,-0.2111971229314804,-0.4830019474029541,0.20642592012882233,0.028950219973921776,-0.24579520523548126,-0.2918934226036072},{-0.23047882318496704,-0.3365252614021301,0.21306103467941284,-0.08063407242298126,0.4603566825389862,0.42294466495513916,-0.18455535173416138,0.14900006353855133,0.20879116654396057,-0.17168772220611572,-0.06058230251073837,-0.27128955721855164,-0.2569766342639923,-0.01240714080631733,0.2088935822248459,-0.06669910252094269},{-0.3087373673915863,-0.02851598896086216,0.46484071016311646,-0.2865231931209564,0.09384407848119736,0.2693136930465698,0.25428706407546997,0.12350451201200485,-0.093263179063797,-0.24461960792541504,0.11696995049715042,0.17837901413440704,-0.2887634336948395,0.1992216855287552,-0.19785232841968536,-0.42390722036361694},{-0.34598249197006226,-0.1348801851272583,0.16463913023471832,0.1626790165901184,0.0915021225810051,-0.22156605124473572,0.26200827956199646,-0.332406222820282,0.2536814510822296,-0.15215575695037842,-0.07265742123126984,-0.20892292261123657,-0.08603829145431519,0.07904435694217682,-0.023753562942147255,0.1589602679014206},{0.006541941314935684,0.2081286758184433,0.23050673305988312,-0.18183012306690216,0.179361030459404,-0.2751193344593048,-0.15102525055408478,-0.2473481446504593,0.06116554141044617,0.2810100018978119,0.20010951161384583,0.019818997010588646,-0.1502658575773239,0.1467164158821106,0.12775178253650665,0.14473912119865417},{-0.09757769107818604,0.1993263214826584,-0.03429952636361122,-0.07544011622667313,-0.21206790208816528,0.204253688454628,-0.3632298409938812,-0.16256046295166016,-0.4322260320186615,-0.3576710522174835,0.12567366659641266,0.020015159621834755,-0.007451467681676149,-0.1961154043674469,0.18349634110927582,-0.06062794476747513}};
static const float actor_encoder_self_encoder_2_weight[16][16] = {{-0.39976242184638977,-0.3477540910243988,-0.19123665988445282,0.31020739674568176,-0.35091280937194824,-0.310153603553772,0.09616177529096603,-0.058732256293296814,0.09317591786384583,-0.43122950196266174,-0.31169962882995605,0.38076311349868774,0.2737047076225281,-0.1609814167022705,-0.15787357091903687,0.2209111899137497},{-0.12441696226596832,-0.23779912292957306,0.3281104862689972,-0.20688636600971222,0.05167735368013382,0.002044633962213993,0.18192020058631897,0.28796911239624023,0.29021701216697693,-0.42246130108833313,-0.33530673384666443,-0.03965437039732933,-0.22282341122627258,-0.1750493347644806,0.16569051146507263,-0.17290890216827393},{0.11002015322446823,-0.3298186957836151,0.4277369976043701,-0.015470526181161404,0.2398485243320465,-0.31165674328804016,0.18141449987888336,-0.40183302760124207,-0.04444858804345131,-0.5479298233985901,0.28461816906929016,0.19410544633865356,-0.3141755759716034,-0.30232539772987366,-0.36305826902389526,-0.028514696285128593},{0.09380609542131424,-0.29389578104019165,-0.03739231452345848,0.4111664891242981,0.10977320373058319,0.4692407250404358,-0.1378175914287567,-0.17730402946472168,0.14384408295154572,0.03349088132381439,-0.10575954616069794,0.2529323101043701,-0.09566038846969604,-0.07864057272672653,0.4049880802631378,0.13479216396808624},{-0.1852564960718155,-0.0022821722086519003,0.19720858335494995,0.3737705945968628,-0.1482527107000351,-0.08364976197481155,0.15639927983283997,-0.15943171083927155,0.29806822538375854,0.24873380362987518,-0.3190181851387024,-0.11425364017486572,0.039394523948431015,0.10035766661167145,-0.41427066922187805,-0.16421444714069366},{-0.2765073776245117,-0.29295915365219116,0.3593025803565979,0.07326989620923996,0.40583762526512146,0.007144628558307886,-0.05722988396883011,0.04944805055856705,-0.2770850658416748,0.11300787329673767,-0.009865513071417809,-0.28267526626586914,0.18521054089069366,-0.1810326874256134,-0.40804323554039,-0.12455002963542938},{-0.3721534311771393,-0.07600834220647812,0.2597092092037201,0.3873734772205353,-0.18614859879016876,-0.382149338722229,0.0958249419927597,0.058089256286621094,-0.09077747911214828,0.39525699615478516,-0.270181268453598,-0.43474093079566956,0.12470584362745285,-0.0711689367890358,-0.34953173995018005,-0.2853471338748932},{0.27000027894973755,-0.013237816281616688,-0.43306276202201843,0.13895352184772491,-0.2730962634086609,-0.3572373390197754,-0.2364719659090042,-0.0058183204382658005,-0.3690192401409149,-0.015089903026819229,0.3522220551967621,0.3453793227672577,-0.3289167284965515,-0.05247538164258003,-0.35517358779907227,-0.4204147160053253},{0.1507844775915146,0.41795486211776733,0.05169413611292839,0.3200140595436096,-0.3867460787296295,-0.0425596721470356,-0.38351261615753174,0.381510853767395,0.14422237873077393,-0.06266045570373535,0.0423642136156559,0.2958303391933441,-0.22427409887313843,-0.3160609304904938,0.09024250507354736,0.23297591507434845},{-0.25225821137428284,-0.21563145518302917,-0.3381846249103546,0.010853759944438934,0.3352101445198059,0.13593870401382446,-0.29683518409729004,-0.13565947115421295,0.23639871180057526,0.3710854947566986,-0.17013202607631683,0.02568967454135418,-0.17374908924102783,-0.05498844385147095,-0.048585012555122375,-0.046422574669122696},{-0.15767209231853485,0.29082122445106506,0.4268646538257599,-0.3862869441509247,0.34957942366600037,0.3871903419494629,-0.25103482604026794,0.017717674374580383,-0.28488218784332275,0.21222954988479614,0.35968533158302307,-0.18203294277191162,-0.1529657244682312,-0.13231775164604187,0.2163703888654709,0.24166248738765717},{-0.39057111740112305,-0.2928085923194885,-0.12471423298120499,-0.14893995225429535,0.00884890928864479,-0.2527112662792206,-0.10627903044223785,0.18813128769397736,0.004854430910199881,0.03318606689572334,0.05147533491253853,0.1298471987247467,-0.33739951252937317,0.05187484994530678,0.026903778314590454,-0.03319987654685974},{0.027758514508605003,-0.33235037326812744,-0.03906833380460739,0.1731860637664795,-0.3419433534145355,0.18987295031547546,0.14287452399730682,0.31510013341903687,0.12493126839399338,0.38226616382598877,0.055362600833177567,0.2255069762468338,0.08748969435691833,-0.3483487665653229,0.016119884327054024,-0.060185015201568604},{-0.14494545757770538,0.0024809609167277813,-0.2297917604446411,0.2092738300561905,-0.4109370708465576,0.0542244054377079,-0.13135980069637299,0.3314978778362274,-0.14448454976081848,-0.20879483222961426,-0.21831633150577545,0.14279504120349884,-0.05212477594614029,0.08607622981071472,-0.13290904462337494,-0.38930627703666687},{0.09764570742845535,-0.2465611696243286,-0.16660097241401672,-0.3962014317512512,-0.1021779403090477,0.15784858167171478,-0.279645174741745,0.013832749798893929,0.3779976963996887,0.12204498052597046,0.37934410572052,-0.09216993302106857,-0.4315950274467468,0.39705726504325867,-0.2823546528816223,0.06080147624015808},{0.05982712283730507,0.13105899095535278,0.1487608104944229,0.22814327478408813,0.3165377378463745,-0.2389903962612152,-0.11177437007427216,-0.1410766839981079,-0.011496154591441154,0.2525465190410614,-0.04520881548523903,-0.2595202922821045,-0.3115098178386688,0.3461798131465912,-0.3555779457092285,0.048627641052007675}};
static const float actor_encoder_feed_forward_0_weight[20][32] = {{-0.07331564277410507,-0.059852004051208496,0.02719903364777565,0.0053960345685482025,0.07832551747560501,-0.012022032402455807,-0.3102784752845764,-0.1719476878643036,0.16428351402282715,-0.2662045955657959,0.20594552159309387,-0.1096416711807251,0.26192164421081543,0.19513653218746185,0.2008064240217209,-0.09719018638134003,-0.04300931841135025,0.1145733967423439,0.3529045879840851,-0.13711988925933838,0.19219480454921722,0.128043532371521,-0.3039448857307434,0.018231041729450226,-0.15324373543262482,0.15635204315185547,0.20545458793640137,-0.2209158092737198,-0.020115720108151436,0.10538216680288315,0.22335954010486603,-0.19649635255336761},{0.10177811980247498,-0.15525951981544495,-0.050712183117866516,-0.16588543355464935,-0.24373480677604675,-0.2765212655067444,0.0828319862484932,-0.04355344921350479,-0.11227577924728394,0.21722912788391113,-0.12543953955173492,0.31374379992485046,-0.31741204857826233,0.19776736199855804,-0.27196669578552246,-0.0837320014834404,0.015449860133230686,0.017559869214892387,0.20698930323123932,0.2975103259086609,0.22150452435016632,-0.008814964443445206,0.22353100776672363,0.25763317942619324,-0.01320707332342863,0.028428753837943077,0.2114550769329071,0.24960455298423767,0.042986199259757996,0.07608908414840698,-0.2903186082839966,-0.03752027824521065},{-0.14609147608280182,-0.11295384913682938,-0.15961001813411713,-0.16872897744178772,-0.12874306738376617,0.3596680164337158,0.014292488805949688,0.022227376699447632,0.08389408141374588,0.03650616854429245,0.07429401576519012,0.04505433514714241,-0.23609431087970734,0.06870065629482269,0.22347408533096313,-0.20909146964550018,0.323028028011322,-0.17328479886054993,-0.17805121839046478,-0.16215066611766815,0.25933071970939636,0.17769716680049896,-0.2628297209739685,-0.08971016108989716,-0.2375979721546173,-0.32185107469558716,0.2296779751777649,0.1912936270236969,0.0742642804980278,-0.05248187482357025,-0.10191775113344193,-0.07639326900243759},{-0.2050360143184662,-0.16352277994155884,-0.1470559537410736,0.2224806845188141,0.2081281542778015,-0.02426341362297535,-0.06749600917100906,-0.08308098465204239,0.06049402803182602,-0.2728729844093323,0.11974698305130005,-0.15645283460617065,0.19471517205238342,0.16272665560245514,0.26337915658950806,-0.14816100895404816,-0.2183515578508377,0.07555870711803436,0.05222073569893837,0.1963915228843689,0.17657624185085297,0.2678174078464508,-0.0012336806394159794,-0.22497814893722534,0.03856067731976509,0.17179138958454132,-0.12633754312992096,-0.302550345659256,-0.09863364696502686,0.05226876586675644,-0.2120453417301178,-0.27743542194366455},{0.08412543684244156,0.19086866080760956,-0.03576575219631195,0.004365869332104921,0.19755113124847412,-0.18652309477329254,0.20443114638328552,0.13075070083141327,0.051007095724344254,-0.06880584359169006,0.2977695167064667,-0.36813971400260925,0.05105981230735779,-0.04116992652416229,-0.2093857377767563,0.18201671540737152,-0.2818482220172882,0.28177574276924133,0.17136874794960022,-0.13040640950202942,-0.10795432329177856,0.10215502232313156,0.2393786609172821,-0.010842542164027691,0.11267416924238205,-0.04054762423038483,0.28159981966018677,0.12550653517246246,0.035006389021873474,0.2189090996980667,0.08768538385629654,0.006933622527867556},{-0.23943445086479187,0.2127678245306015,-0.25377845764160156,0.18540678918361664,0.4572180211544037,-0.4131074547767639,-0.29987648129463196,0.3026036024093628,0.16502055525779724,0.05178556591272354,-0.21599654853343964,-0.13262195885181427,-0.09779468178749084,0.1863424926996231,0.059534214437007904,-0.12362980842590332,-0.37348154187202454,-0.2371761053800583,-0.17070509493350983,0.03209379315376282,0.007824687287211418,0.35930344462394714,-0.0554601289331913,-0.1600351780653,-0.019413666799664497,-0.13457545638084412,-0.10594729334115982,0.06731491535902023,0.22085212171077728,0.20406699180603027,-0.11477786302566528,0.32850509881973267},{-0.26603129506111145,0.2533785104751587,-0.18385720252990723,0.19671882688999176,-0.15307128429412842,-0.2293243706226349,-0.1883097141981125,-0.06080364063382149,0.011207720264792442,-0.04243363067507744,0.35120445489883423,-0.23191291093826294,-0.09602615982294083,0.09889813512563705,-0.20049436390399933,-0.2013765424489975,0.2572810649871826,-0.24663567543029785,-0.17154784500598907,-0.2684682011604309,0.2830170691013336,-0.39723360538482666,-0.2261027693748474,0.1846281737089157,0.2402230054140091,0.16890652477741241,-0.1060376688838005,0.0946788489818573,-0.12315665185451508,-0.14672508835792542,-0.17934367060661316,0.10004264861345291},{-0.3649977147579193,-0.06289097666740417,-0.16379782557487488,-0.05799224600195885,0.04587043076753616,0.090383380651474,0.2795746326446533,0.21105724573135376,-0.07113926112651825,-0.24830767512321472,-0.09956766664981842,0.06460422277450562,-0.2801085412502289,0.18171603977680206,-0.026945076882839203,0.23272927105426788,-0.2724843919277191,-0.34614527225494385,-0.2263372242450714,-0.028637360781431198,0.16233740746974945,-0.21378754079341888,-0.30493202805519104,-0.07139740884304047,-0.22127847373485565,0.021972492337226868,0.16548006236553192,0.008726662956178188,-0.33730024099349976,-0.11343929171562195,0.2580512464046478,-0.1307305246591568},{-0.10171779245138168,0.05063127353787422,-0.008408447727560997,0.010097958147525787,-0.2503208518028259,-0.15630635619163513,0.19993682205677032,-0.04455220326781273,0.16252674162387848,0.13114453852176666,0.1517600566148758,0.30955296754837036,-0.34538939595222473,-0.012908331118524075,0.024720745161175728,0.035819657146930695,-0.1115267351269722,0.07705770432949066,0.11552922427654266,0.2568863332271576,-0.09257151186466217,-0.038596078753471375,0.011180950328707695,0.18092045187950134,-0.07561717182397842,-0.14425167441368103,0.30527639389038086,0.05538881570100784,0.08671512454748154,0.10466519743204117,0.08334308117628098,0.2517966330051422},{-0.25952503085136414,0.10765888541936874,-0.23333652317523956,-0.23382331430912018,0.08536311984062195,-0.18764303624629974,0.2762277126312256,0.21052837371826172,0.17867611348628998,-0.05719248577952385,-0.1769501119852066,-0.3623825013637543,-0.02089795097708702,-0.3722749948501587,0.13457854092121124,0.24349410831928253,0.14468972384929657,-0.12915174663066864,0.03254106640815735,-0.31979185342788696,0.3752872347831726,0.2712481915950775,-0.10243449360132217,0.13775962591171265,-0.1834162324666977,0.1395738422870636,-0.34187474846839905,0.060028646141290665,-0.024864990264177322,-0.10369037836790085,-0.34164080023765564,-0.12399668991565704},{0.23128995299339294,-0.22354166209697723,0.19360558688640594,0.290544718503952,-0.18363173305988312,-0.19566066563129425,-0.13575875759124756,0.17371350526809692,0.13477782905101776,0.34214502573013306,-0.2592962682247162,0.184139221906662,0.22305910289287567,0.0224715955555439,-0.08905371278524399,0.3096573054790497,-0.17671914398670197,-0.1267581284046173,0.18175235390663147,0.18399031460285187,-0.2589544355869293,-0.03974959999322891,0.06817261874675751,0.19163461029529572,0.26377570629119873,-0.17895609140396118,0.27636775374412537,-0.2843361794948578,-0.0675002858042717,0.10990408807992935,-0.029220564290881157,-0.10581129789352417},{0.2713595926761627,0.14621126651763916,0.2032850831747055,-0.11414405703544617,0.1826120764017105,0.2138945311307907,-0.3383343517780304,-0.13228005170822144,0.18766067922115326,-0.2731451690196991,-0.29957348108291626,-0.007897456176578999,0.08896051347255707,0.14475709199905396,-0.2660776674747467,-0.032161030918359756,0.34219521284103394,0.2878705561161041,-0.1356862634420395,-0.22944770753383636,0.22492891550064087,0.04274803400039673,0.0790695920586586,-0.022211018949747086,0.3441755771636963,0.2722785472869873,0.017697744071483612,-0.20559477806091309,0.13679435849189758,-0.3102492392063141,-0.1732257455587387,0.17083878815174103},{0.022912342101335526,-0.14207322895526886,-0.1817561388015747,0.17253892123699188,0.051887836307287216,-0.034886814653873444,-0.1757027953863144,-0.06385403871536255,-0.15277893841266632,-0.1280285120010376,0.2263081669807434,-0.33693647384643555,0.030463295057415962,0.1950269639492035,0.12061480432748795,0.05182783305644989,-0.16265958547592163,-0.11779415607452393,0.26631468534469604,0.05919470265507698,0.27353209257125854,0.19746139645576477,-0.006920675281435251,0.11983533948659897,0.28082332015037537,0.3350328207015991,0.0016294508241117,0.23894773423671722,0.07224806398153305,-0.11693982779979706,0.02212040312588215,0.34534522891044617},{0.2507951855659485,-0.21248199045658112,0.16813457012176514,0.08264816552400589,0.09318297356367111,0.27423056960105896,-0.16684643924236298,0.16589729487895966,0.013878632336854935,0.01559722051024437,-0.060121502727270126,-0.32693904638290405,-0.08129701763391495,0.1221933364868164,0.1838819682598114,0.2737133800983429,0.09174306690692902,-0.1113053485751152,0.10822928696870804,-0.2702758014202118,-0.2400992512702942,-0.2972255051136017,0.21783052384853363,-0.08120028674602509,0.3101865351200104,0.16666091978549957,-0.31771448254585266,-0.13220445811748505,-0.2936040759086609,0.17793390154838562,0.20827382802963257,0.18608860671520233},{0.11602210253477097,-0.1332794725894928,-0.24314561486244202,0.27293235063552856,0.1650029867887497,0.23530660569667816,0.16457396745681763,-0.19269248843193054,0.003502595005556941,-0.22023285925388336,0.20696794986724854,0.2029470056295395,0.29916509985923767,0.03691202402114868,-0.044832002371549606,-0.29237475991249084,0.07770859450101852,0.037842441350221634,-0.19749502837657928,-0.15946492552757263,-0.27900734543800354,0.10293536633253098,-0.02703060396015644,-0.2345736026763916,0.17137174308300018,0.17321905493736267,0.015704743564128876,0.14340944588184357,-0.22075162827968597,-0.2652082145214081,-0.14720910787582397,0.2973273992538452},{-0.2529250383377075,-0.3853786885738373,-0.027549706399440765,-0.21349374949932098,-0.017031820490956306,-0.3137573301792145,-0.017516709864139557,0.06052295118570328,0.23283307254314423,0.1964227706193924,0.1187637597322464,-0.15751472115516663,-0.01683114469051361,0.32401880621910095,0.33938053250312805,-0.08444982767105103,0.038184892386198044,-0.21796320378780365,-0.09428147226572037,-0.27125465869903564,0.18427664041519165,-0.1092156246304512,-0.24213752150535583,0.03150692954659462,-0.24628081917762756,-0.40053999423980713,-0.04541923105716705,0.3663890063762665,-0.10943959653377533,0.267331063747406,0.0739986002445221,-0.26208075881004333},{-0.019056344404816628,-0.0063878376968204975,0.3015924394130707,0.02746015228331089,0.012649722397327423,0.2267311066389084,-0.08933659642934799,-0.2155265212059021,-0.2679865062236786,0.2559332251548767,0.130879744887352,0.16673462092876434,0.09570198506116867,-0.10564678907394409,0.27930063009262085,0.06327655166387558,0.2448180466890335,-0.10228575766086578,0.1777942180633545,-0.11135304719209671,0.28008803725242615,-0.24934417009353638,-0.23292982578277588,-0.22662551701068878,-0.2129276692867279,-0.19107159972190857,-0.20776329934597015,-0.33265984058380127,-0.11371392756700516,-0.22197861969470978,0.19736206531524658,0.0646703913807869},{-0.07523028552532196,0.07289963215589523,0.0634562149643898,0.12678366899490356,0.011789831332862377,-0.03223392739892006,0.357929527759552,0.24668475985527039,-0.18384671211242676,0.3301578462123871,-0.06233006343245506,0.03553617745637894,0.0300301406532526,0.29228726029396057,-0.09273366630077362,-0.030843742191791534,0.24109196662902832,0.18872743844985962,0.07172676920890808,0.029062656685709953,0.09112320095300674,0.038024693727493286,0.11539402604103088,-0.29066792130470276,0.05775371566414833,-0.3274499475955963,-0.15491724014282227,-0.1211026981472969,-0.034429606050252914,0.3634611964225769,-0.2184060513973236,-0.20739448070526123},{-0.20055006444454193,-0.01108287088572979,-0.3427064120769501,0.052260592579841614,-0.29950204491615295,0.2918354272842407,0.25790315866470337,0.13791589438915253,-0.07500772178173065,-0.23811182379722595,-0.11181260645389557,0.18023079633712769,0.03659430146217346,0.15322153270244598,0.0336025208234787,0.08069636672735214,-0.17119601368904114,0.05376362428069115,-0.05262008309364319,-0.22942623496055603,-0.16600339114665985,-0.07854556292295456,-0.20921720564365387,0.10251037031412125,-0.2620130181312561,-0.14278270304203033,-0.006010155659168959,0.2353934496641159,-0.24949462711811066,0.08645064383745193,-0.08893053233623505,0.030431605875492096},{0.09967492520809174,0.2420254349708557,0.3060404658317566,0.03541744127869606,0.019826829433441162,0.15375183522701263,-0.19385769963264465,-0.08141625672578812,-0.29264035820961,0.2234324812889099,-0.1654175966978073,-0.2511957883834839,0.0772581398487091,0.22223643958568573,0.058127909898757935,0.1796799749135971,0.2414272576570511,-0.24172565340995789,-0.23335526883602142,0.09816774725914001,0.15513825416564941,-0.20104967057704926,0.08012483268976212,0.3139694631099701,0.22364357113838196,-0.256063848733902,-0.10092516988515854,-0.04538106545805931,0.21947215497493744,-0.1917060762643814,-0.04676385223865509,0.2883000075817108}};
static const float action_parameterization_distribution_linear_weight[32][4] = {{0.07154060900211334,-0.19240939617156982,0.3412839472293854,-0.2079862803220749},{-0.20447689294815063,0.03505156189203262,0.2897869944572449,0.2119622677564621},{0.1703117936849594,-0.3803282678127289,-0.2834693491458893,0.29918012022972107},{-0.24165235459804535,0.19137366116046906,0.03433489426970482,-0.12241383641958237},{0.24027499556541443,0.1187470331788063,0.2253616899251938,0.09477607160806656},{0.08877366036176682,0.0008641468011774123,-0.3539883494377136,0.18544362485408783},{-0.0716945230960846,-0.01096314936876297,0.2579866349697113,0.4111965000629425},{0.18382133543491364,0.3481462597846985,-0.2220236212015152,0.3082709312438965},{0.2716480493545532,0.24577759206295013,-0.281701922416687,-0.004814907442778349},{0.2533654570579529,0.12711074948310852,-0.14858238399028778,0.054960716515779495},{0.22225554287433624,0.10915615409612656,-0.17563888430595398,0.01653124764561653},{-0.28809598088264465,-0.23243968188762665,-0.11974179744720459,-0.21938395500183105},{0.03961508721113205,0.1490466296672821,0.0880003422498703,-0.028456702828407288},{0.25791990756988525,-0.22115129232406616,0.09960819035768509,-0.06704483181238174},{0.05327201262116432,0.1573038250207901,0.11510761082172394,-0.2357763946056366},{0.2582973539829254,-0.23655498027801514,0.2745846211910248,0.06905658543109894},{-0.23627260327339172,-0.07257308065891266,-0.2620484232902527,-0.09576532989740372},{0.26852497458457947,0.2272930145263672,-0.3405574858188629,-0.13211743533611298},{-0.054236434400081635,0.15662942826747894,-0.2619340717792511,-0.04878798872232437},{-0.27423277497291565,0.2383776307106018,-0.1587180197238922,-0.2554776966571808},{-0.0811137706041336,0.08573362976312637,-0.1390080451965332,0.28309857845306396},{0.33890458941459656,0.16607101261615753,0.19115430116653442,0.23739108443260193},{-0.07054971903562546,0.0634879320859909,-0.257046103477478,0.28310444951057434},{-0.10118367522954941,0.2432546317577362,-0.2879372537136078,-0.31779950857162476},{-0.1537928283214569,0.2288336455821991,0.17663894593715668,-0.3090144395828247},{-0.20940539240837097,0.12367884069681168,-0.20523694157600403,0.2603103518486023},{0.02616780996322632,-0.15033194422721863,0.049435246735811234,-0.1378067582845688},{0.22321894764900208,-0.1800466775894165,-0.2755917012691498,0.027225984260439873},{0.07101202011108398,-0.08615953475236893,0.07701425999403,0.1394461840391159},{0.02787131257355213,0.1408054679632187,0.2253628671169281,0.024518586695194244},{-0.014740829356014729,-0.2316451519727707,-0.3138936460018158,-0.05301358923316002},{-0.2537386417388916,0.11686890572309494,0.07394606620073318,0.21016079187393188}};
static const float actor_encoder_self_encoder_0_bias[16] = {0.07663518190383911,-0.00170690577942878,0.05656592547893524,0.030536174774169922,-0.019985437393188477,-0.04167189449071884,-0.05504263937473297,-0.13224045932292938,0.07658317685127258,0.12206125259399414,-0.03824930265545845,-0.02323550172150135,-0.014143940061330795,0.11583681404590607,-0.027147116139531136,-0.032797493040561676};
static const float actor_encoder_self_encoder_2_bias[16] = {0.01530387345701456,-0.02930275909602642,0.056117329746484756,0.05960363522171974,0.020437076687812805,0.07450559735298157,-0.0007118148496374488,0.014978601597249508,0.040146686136722565,0.011082771234214306,-0.013888629153370857,-0.024716733023524284,0.0023932836484164,0.019991792738437653,0.01109230425208807,0.07468027621507645};
static const float actor_encoder_feed_forward_0_bias[32] = {-0.0036349815782159567,0.02975120022892952,0.0035504254046827555,-0.04086028411984444,0.05793139338493347,0.011486841365695,0.06200295314192772,0.0075940475799143314,0.0072659580036997795,0.016201885417103767,0.009092830121517181,-0.029132511466741562,0.04153135418891907,-0.0003773456846829504,-0.004020035266876221,0.00041992319165728986,-0.06438138335943222,0.0010935660684481263,-0.013156491331756115,-0.019551126286387444,0.04955562949180603,0.04319143295288086,-0.0023105815052986145,-0.03875555470585823,-0.00899424683302641,-0.011414344422519207,-0.004851156380027533,0.006765583995729685,0.048089876770973206,0.06803324818611145,-0.024608837440609932,0.011853475123643875};
static const float action_parameterization_distribution_linear_bias[4] = {0.018343379721045494,0.023921236395835876,0.032348450273275375,0.03763381019234657};
static const float actor_encoder_obstacle_encoder_0_weight[16][4] = {{-0.1819053441286087,-0.546596348285675,0.20757222175598145,-0.5648160576820374},{0.02864215523004532,-0.46485111117362976,-0.23044633865356445,-0.35862812399864197},{0.12555113434791565,-0.7331461310386658,-0.042397238314151764,0.36971771717071533},{-0.2795502245426178,-0.3724175691604614,0.005825918633490801,0.07013502717018127},{0.18178711831569672,0.12060938030481339,0.04515595734119415,0.16150625050067902},{0.5070134401321411,-0.15053574740886688,0.21657586097717285,-0.0008570185746066272},{0.19561097025871277,-0.639475405216217,-0.09968892484903336,-0.5162746906280518},{0.5767498016357422,-0.5875889658927917,-0.05589710548520088,-0.3229605555534363},{0.413341224193573,0.06540101766586304,-0.09198525547981262,-0.5914005041122437},{-0.3198290765285492,-0.58384108543396,-0.49558553099632263,-0.4923804998397827},{-0.15840938687324524,-0.5195122957229614,-0.2164955884218216,-0.02618386410176754},{-0.058295536786317825,-0.36953234672546387,0.4052889049053192,-0.31044071912765503},{0.27132749557495117,0.45644474029541016,-0.5975349545478821,0.3574366271495819},{0.07952218502759933,-0.24283099174499512,-0.5241259336471558,-0.5807461142539978},{0.5378889441490173,-0.27912500500679016,-0.7062451243400574,-0.6936010718345642},{0.17877928912639618,-0.4004611372947693,-0.09089638292789459,0.3649309575557709}};
static const float actor_encoder_obstacle_encoder_2_weight[4][4] = {{-0.677924633026123,0.6534343957901001,-0.6226750016212463,-0.18880870938301086},{0.7157352566719055,-0.2192038595676422,0.13202638924121857,0.2538785934448242},{0.16043402254581451,-0.3278156518936157,-0.9109143018722534,-0.27073362469673157},{-0.8270300030708313,-0.6640906929969788,0.24579615890979767,0.2647237479686737}};
static const float actor_encoder_obstacle_encoder_0_bias[4] = {0.15553490817546844,-0.10767235606908798,-0.08251781761646271,-0.08636005222797394};
static const float actor_encoder_obstacle_encoder_2_bias[4] = {0.009742889553308487,0.01155867800116539,0.048769183456897736,0.04054689034819603};
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
