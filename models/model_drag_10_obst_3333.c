
#include <random>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cstring> 


typedef struct control_t_n {
	float thrust_0;
	float thrust_1;
	float thrust_2;
	float thrust_3;
} control_t_n;

void networkEvaluate(control_t_n* control_n, const float* state_array);

static const int NEIGHBORS = 0;
static const int NBR_DIM = 0; 

static const int NUM_OBSTACLES = 2; 
static const int OBST_DIM = 8;


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
static const float actor_encoder_self_encoder_0_weight[18][16] = {{0.4170542359352112,0.3252270519733429,0.13626763224601746,-0.08138331025838852,-0.06402014195919037,-0.0051387823186814785,0.12425202131271362,0.11345997452735901,-0.12466000765562057,0.11562696844339371,-0.1525949239730835,-0.007122637238353491,0.02029038779437542,0.08898106962442398,0.25058555603027344,0.22496844828128815},{0.0856332927942276,0.19576846063137054,-0.00909245852380991,0.19084060192108154,-0.3181377947330475,0.1703689694404602,-0.2175920605659485,-0.0533059723675251,0.30480289459228516,-0.16240748763084412,-0.2945224940776825,-0.3289357125759125,0.11214815825223923,-0.13939906656742096,0.3691624402999878,-0.12604159116744995},{-0.043502382934093475,-0.06501749157905579,0.004016281571239233,0.6638889312744141,-0.09952893853187561,-0.6480698585510254,0.08530815690755844,0.2391349971294403,-0.14135748147964478,0.34566694498062134,-0.11444445699453354,-0.41348329186439514,0.2674735188484192,0.28627079725265503,0.25327804684638977,-0.22532200813293457},{0.29406964778900146,-0.2608559727668762,-0.0833808109164238,0.10005161911249161,-0.3214692175388336,0.19190852344036102,0.4360545873641968,0.03200056031346321,-0.26074448227882385,0.2703764736652374,0.11128801107406616,0.018361395224928856,-0.03387616574764252,-0.19014106690883636,0.14222352206707,0.1179347112774849},{0.157809779047966,0.11446633189916611,0.18552039563655853,0.13944539427757263,-0.06088119000196457,0.2759370803833008,-0.12430692464113235,-0.03587348014116287,0.23427490890026093,-0.2020905762910843,-0.13174980878829956,0.3788183331489563,-0.3334217071533203,-0.06068825721740723,0.5872423648834229,-0.1612558513879776},{0.15965723991394043,0.2513540983200073,-0.1058279275894165,0.16971342265605927,-0.2430780529975891,0.15479779243469238,0.24748313426971436,-0.22619345784187317,-0.2293211668729782,0.09860097616910934,0.15525661408901215,-0.4299635589122772,-0.0183149054646492,0.4779230058193207,0.3482230007648468,-0.27662625908851624},{0.11608186364173889,0.17783857882022858,0.16591604053974152,-0.18739673495292664,0.2766810655593872,-0.2972906231880188,0.5299296379089355,-0.10513831675052643,0.042820148169994354,0.4257534444332123,0.13852721452713013,0.18013563752174377,0.03744639456272125,0.17002680897712708,0.16586482524871826,-0.31568485498428345},{-0.29631438851356506,0.3798195421695709,0.21912549436092377,0.22067618370056152,-0.11460931599140167,0.3309028148651123,-0.24872487783432007,-0.12017472833395004,0.05337272584438324,-0.20168627798557281,0.13802117109298706,-0.006929316557943821,-0.2388223111629486,-0.12116057425737381,-0.17176181077957153,0.2694541811943054},{0.2506882846355438,0.4305533766746521,-0.31616929173469543,0.4321223497390747,-0.3790188431739807,0.2558588683605194,0.18822473287582397,0.4014159142971039,-0.26882216334342957,0.13880547881126404,0.23041333258152008,0.24617087841033936,-0.18156735599040985,-0.5481663942337036,-0.21826106309890747,0.5969123840332031},{-0.23351064324378967,0.157961905002594,-0.2906668484210968,-0.28941112756729126,0.3668365776538849,-0.04820531979203224,-0.22095546126365662,0.13628482818603516,0.3749730587005615,0.2850346863269806,-0.2027091681957245,-0.15801186859607697,0.20376363396644592,0.19633184373378754,-0.34583672881126404,-0.11040981858968735},{0.4379488229751587,-0.19607697427272797,0.5092923045158386,-0.3396284282207489,0.26928719878196716,0.00014695774007122964,0.41621527075767517,-0.011374223977327347,0.3616941273212433,-0.16836988925933838,0.026410304009914398,0.10510236024856567,-0.15599331259727478,-0.03737815469503403,0.2859537601470947,-0.06802918761968613},{-0.14881400763988495,-0.10901843756437302,0.4271538257598877,0.4541853666305542,-0.5634064674377441,0.06441054493188858,0.10229405015707016,0.3168941140174866,0.040379755198955536,0.1345575600862503,-0.43458065390586853,-0.22483137249946594,-0.6415891647338867,0.09976613521575928,-0.11112013459205627,-0.3842891454696655},{-0.10949042439460754,0.5077344179153442,-0.1927739530801773,0.30039218068122864,0.005571590270847082,-0.4582503139972687,-0.15220849215984344,-0.3754729628562927,-0.37692946195602417,0.638371467590332,-0.10548257827758789,0.4194018244743347,0.681288480758667,-0.3478941321372986,-0.31694746017456055,-0.05394790321588516},{0.318800687789917,0.10485000908374786,-0.1263105720281601,0.2439272552728653,-0.43706780672073364,0.011096330359578133,0.5239923596382141,-0.10435468703508377,0.04177607595920563,0.7186864018440247,-0.4673919975757599,-0.4006425142288208,-0.051706328988075256,-0.662116527557373,0.37820982933044434,0.08465847373008728},{-0.010922791436314583,-0.3137570023536682,-0.3853124976158142,0.1641293615102768,-0.3254318833351135,0.1964629590511322,0.1812860667705536,0.2847042977809906,-0.09360911697149277,-0.13729697465896606,-0.4027140438556671,0.39968323707580566,-0.03968457877635956,0.433029443025589,-0.042482901364564896,0.2983911633491516},{0.310372531414032,-0.37161311507225037,-0.1769285798072815,0.10473666340112686,-0.08237529546022415,0.40112799406051636,0.37847697734832764,0.18878303468227386,-0.25401735305786133,0.1689595729112625,-0.3799665868282318,-0.27975475788116455,0.13882486522197723,-0.205220028758049,0.20112080872058868,0.19156886637210846},{0.2862984240055084,-0.13892608880996704,0.13925892114639282,-0.08884039521217346,-0.2909061014652252,0.24763289093971252,-0.24802447855472565,0.15746907889842987,-0.28342631459236145,-0.14079828560352325,0.3171614408493042,-0.3949907720088959,-0.12453389912843704,0.18948084115982056,0.2745281755924225,-0.09606238454580307},{-0.07824341952800751,0.2905972898006439,-0.12297070026397705,0.29152071475982666,-0.3047807812690735,0.39673370122909546,0.3636433780193329,-0.38389167189598083,-0.22996710240840912,-0.3141952455043793,-0.12578149139881134,0.2950148284435272,0.2385905236005783,0.2336008995771408,0.3466419577598572,-0.09321333467960358}};
static const float actor_encoder_self_encoder_2_weight[16][16] = {{0.1888999342918396,0.3110416829586029,0.35941267013549805,-0.46032848954200745,0.35662901401519775,0.12585926055908203,0.05359538644552231,-0.2889065742492676,-0.12745778262615204,-0.06076537445187569,0.08941783010959625,-0.06312821805477142,-0.30387482047080994,0.377850741147995,-0.2999071180820465,0.45322883129119873},{0.37920624017715454,0.047046344727277756,0.3537324368953705,0.402192622423172,-0.27654799818992615,-0.030904920771718025,-0.06954098492860794,0.30555734038352966,-0.03165421634912491,-0.04188193008303642,-0.29750722646713257,0.042118754237890244,0.13363315165042877,0.21984079480171204,-0.006465578451752663,0.12766632437705994},{-0.23566588759422302,-0.08528172969818115,0.0315096452832222,0.22701476514339447,0.25275349617004395,-0.08486109972000122,0.3547062277793884,-0.08758297562599182,-0.041492413729429245,0.1129964143037796,0.055087607353925705,-0.27806970477104187,0.1619274765253067,0.2859882712364197,-0.11382267624139786,-0.11014144122600555},{-0.10463757067918777,0.3053112328052521,0.26984378695487976,-0.26410719752311707,0.03783531114459038,-0.533953070640564,-0.05844511091709137,0.07323969155550003,0.2218090146780014,-0.5295069813728333,-0.13273684680461884,-0.30387604236602783,0.2590976357460022,0.2202911674976349,-0.2533208429813385,0.23766979575157166},{-0.3567779064178467,-0.2057514190673828,0.09281802922487259,-0.2631436586380005,0.3860568106174469,-0.09819365292787552,0.19804193079471588,0.1233971118927002,0.3046044707298279,0.48719748854637146,-0.20518185198307037,0.2915547788143158,-0.2600739002227783,-0.2031937837600708,-0.39095163345336914,0.3229096233844757},{0.44207093119621277,0.14473609626293182,0.1786312758922577,0.17888405919075012,-0.034484609961509705,-0.3282817304134369,0.047090861946344376,0.42223861813545227,-0.29063642024993896,0.44121935963630676,-0.33413970470428467,-0.32238873839378357,0.22427065670490265,-0.41591310501098633,0.20197750627994537,0.12858231365680695},{0.28057408332824707,-0.33726298809051514,0.228042334318161,-0.38116326928138733,0.3543385863304138,0.3169398307800293,-0.1011192724108696,0.11943269520998001,0.09924685209989548,-0.044100768864154816,0.005090662278234959,0.14677082002162933,-0.21295592188835144,0.2418503314256668,0.19195550680160522,-0.11538249999284744},{-0.31268998980522156,0.1326894611120224,-0.24916858971118927,-0.14848771691322327,0.21644175052642822,0.30049794912338257,-0.11687236279249191,-0.3413975238800049,-0.0676446482539177,0.14444465935230255,0.3304363787174225,-0.2837679386138916,0.1297026425600052,0.025901969522237778,0.36215874552726746,0.18751095235347748},{-0.13103696703910828,-0.11585263162851334,0.07701361924409866,0.35833945870399475,0.31916698813438416,-0.3384852707386017,0.26281097531318665,-0.20871922373771667,-0.00402611680328846,0.427329421043396,0.12416330724954605,-0.009554004296660423,-0.1305813193321228,0.12420778721570969,0.27413854002952576,-0.03482078015804291},{0.06864921003580093,-0.33950990438461304,0.29276609420776367,-0.28815314173698425,0.25463828444480896,0.14423556625843048,-0.18700620532035828,0.11829453706741333,0.40079987049102783,-0.011755988933146,0.04631480947136879,0.06125504523515701,0.37928110361099243,0.22792531549930573,-0.4187681972980499,0.47458598017692566},{-0.06650649011135101,0.24950553476810455,-0.24021857976913452,-0.35718008875846863,-0.4005603790283203,0.028023384511470795,0.38159996271133423,-0.4002745747566223,-0.4294891655445099,-0.17212636768817902,-0.22594547271728516,0.013529113493859768,0.2994658946990967,0.2670542597770691,-0.39352452754974365,0.3998866677284241},{0.3052060604095459,0.24962878227233887,0.11677736043930054,-0.19789442420005798,0.1583213359117508,0.0448029451072216,-0.1533287614583969,0.458600252866745,-0.31092870235443115,-0.37215691804885864,-0.3448195457458496,-0.1968507617712021,0.18570810556411743,-0.0564110167324543,-0.08449344336986542,0.10469551384449005},{-0.3968345522880554,-0.0850786343216896,0.2633379399776459,0.14835447072982788,0.3253730833530426,0.00994077231734991,-0.021030070260167122,0.4237760901451111,-0.2772950232028961,-0.36999499797821045,-0.3416416645050049,0.3173355758190155,-0.32994794845581055,0.19506564736366272,-0.019705649465322495,0.3785187900066376},{-0.2005678415298462,0.019172176718711853,-0.3864738643169403,-0.42040613293647766,-0.2840399742126465,-0.12133029103279114,-0.2840849757194519,0.30629196763038635,-0.29347822070121765,-0.43969258666038513,-0.04067893326282501,0.2778564989566803,-0.388079971075058,0.3372047543525696,-0.23101772367954254,-0.19051674008369446},{0.2625751793384552,-0.3441888093948364,-0.03378656134009361,-0.17675712704658508,-0.2473413497209549,-0.19708725810050964,-0.4421595335006714,0.13364636898040771,0.3501824140548706,-0.0895284041762352,0.04102736711502075,0.20717178285121918,-0.06071065738797188,0.1846199631690979,0.06206509470939636,-0.16610504686832428},{0.33286628127098083,0.2873073220252991,0.10939312726259232,0.10818777978420258,0.1291588544845581,0.3922707140445709,0.28572648763656616,0.1961495578289032,-0.1776982694864273,0.06103954091668129,0.2885437607765198,0.0026828672271221876,0.4474128186702728,-0.28536921739578247,-0.024391867220401764,-0.12661956250667572}};
static const float actor_encoder_feed_forward_0_weight[20][32] = {{0.20335236191749573,-0.32420775294303894,0.08543483912944794,0.12600404024124146,-0.05511632189154625,-0.374118447303772,-0.039001014083623886,-0.24970361590385437,0.23310720920562744,0.34727242588996887,-0.002118055708706379,-0.009360536001622677,-0.17652399837970734,-0.2095864862203598,-0.046095289289951324,-0.13270223140716553,0.25444409251213074,0.3628767430782318,-0.0709901973605156,-0.0966905802488327,0.08407534658908844,0.23909789323806763,-0.10433410108089447,-0.19217991828918457,0.19611790776252747,-0.22250382602214813,0.3196895122528076,-0.09755000472068787,0.2618322968482971,-0.21374563872814178,-0.06230900436639786,0.0029295580461621284},{-0.17693080008029938,-0.27415603399276733,0.24928836524486542,0.2898302972316742,-0.13253465294837952,-0.10980255901813507,0.09393282234668732,-0.11392279714345932,-0.19504302740097046,-0.20974473655223846,-0.11220700293779373,-0.05263659358024597,0.29085755348205566,0.2768521308898926,0.0499548502266407,-0.19217237830162048,-0.080381378531456,-0.13759014010429382,-0.22580693662166595,0.206328347325325,-0.03995608910918236,-0.42674416303634644,-0.0667666345834732,-0.20305924117565155,-0.1641104519367218,-0.20798085629940033,0.26906776428222656,0.4390413165092468,-0.18448908627033234,-0.2117183357477188,0.1093561053276062,-0.22649335861206055},{-0.10198531299829483,0.09742525219917297,0.09006627649068832,-0.2242382913827896,0.14558017253875732,0.2240786850452423,-0.23474358022212982,-0.323574036359787,0.03414260223507881,-0.11061424762010574,-0.03440917655825615,0.041388049721717834,-0.08149827271699905,-0.29542872309684753,0.19852834939956665,0.15609897673130035,-0.22539576888084412,-0.09142071008682251,-0.26326408982276917,0.22369521856307983,0.16512618958950043,-0.07868555188179016,0.3544140160083771,-0.26581722497940063,-0.18978658318519592,-0.1606617271900177,-0.20842240750789642,0.2277791053056717,-0.0705651342868805,-0.09263251721858978,0.0345185287296772,-0.18724095821380615},{0.36676913499832153,-0.09732691198587418,-0.22118373215198517,0.13935378193855286,0.15487335622310638,-0.0874861553311348,0.1565113663673401,-0.3357917368412018,0.30944204330444336,0.015534492209553719,-0.015018301084637642,-0.23888349533081055,-0.29734158515930176,0.06475688517093658,-0.10204890370368958,0.03062416799366474,0.19833433628082275,0.28935447335243225,0.1742325723171234,0.19626648724079132,0.026408426463603973,-0.08703988790512085,-0.09159134328365326,-0.07884548604488373,0.14199674129486084,0.15932251513004303,0.20968778431415558,0.004891917109489441,-0.04680299386382103,0.0018955855630338192,0.05755668133497238,-0.22136655449867249},{0.07143308222293854,-0.003537209238857031,-0.23358118534088135,-0.35782796144485474,-0.0764954686164856,0.1902880221605301,-0.19607901573181152,0.31240177154541016,-0.24548932909965515,0.2115054726600647,0.08723863214254379,0.15482650697231293,-0.19224047660827637,0.15377391874790192,0.32963213324546814,0.18936659395694733,-0.008435853756964207,-0.047914616763591766,-0.08270283788442612,-0.08110720664262772,0.2980254888534546,-0.10089033842086792,0.04495542123913765,0.24586434662342072,0.1439885050058365,-0.25474250316619873,-0.11393076926469803,-0.07383327186107635,0.18094466626644135,0.1607319861650467,-0.005164206959307194,-0.3136744797229767},{0.20160937309265137,-0.20114514231681824,0.10818592458963394,-0.13019990921020508,0.15873749554157257,0.14511796832084656,0.06362714618444443,0.39661574363708496,-0.0422029085457325,-0.15654465556144714,-0.20417694747447968,-0.2432982176542282,0.20149892568588257,-0.07167855650186539,0.2058633267879486,0.17568713426589966,0.12178388983011246,0.3206924498081207,-0.03663105145096779,0.3346436321735382,-0.03675023093819618,-0.15509693324565887,-0.16634610295295715,0.06871926039457321,-0.048562031239271164,-0.2662206292152405,-0.3626648485660553,-0.2843344211578369,-0.005859781987965107,-0.13827846944332123,-0.14740557968616486,0.10244834423065186},{-0.13074690103530884,0.3055821657180786,0.07500698417425156,0.28491172194480896,-0.29624736309051514,0.08252255618572235,-0.12933647632598877,0.0815659612417221,-0.13978517055511475,0.13507232069969177,-0.1550142914056778,0.2240867167711258,-0.33478671312332153,0.3057427704334259,0.08770846575498581,0.05453866720199585,0.08910422027111053,-0.3397846221923828,-0.19813862442970276,-0.047498658299446106,0.09327644854784012,0.057578474283218384,-0.23388245701789856,0.18028797209262848,0.32812440395355225,-0.2621403932571411,-0.2985382080078125,0.06454433500766754,0.2838291525840759,-0.04520225152373314,0.24674096703529358,-0.12955628335475922},{-0.23049871623516083,0.025932423770427704,0.06107298284769058,-0.04608947038650513,-0.041692011058330536,0.18550550937652588,0.04235261306166649,0.3093182444572449,0.006124129053205252,-0.267132043838501,0.22218254208564758,0.030977588146924973,-0.11143597215414047,0.2788406014442444,0.08505567163228989,-0.1507757306098938,-0.07535146921873093,0.3170067369937897,-0.12317004054784775,-0.05142045021057129,-0.04669404774904251,-0.3097917437553406,0.14147616922855377,-0.12818017601966858,-0.12232392281293869,0.14035065472126007,0.06696948409080505,0.2091989517211914,0.07398205250501633,-0.18502861261367798,-0.30193030834198,0.0921005830168724},{-0.1835649311542511,-0.18257728219032288,-0.3848521113395691,-0.3278065025806427,0.1606552004814148,0.16993193328380585,0.10663123428821564,-0.13671615719795227,0.24734292924404144,0.16778315603733063,-0.05603611096739769,-0.20024777948856354,-0.19497717916965485,-0.40733811259269714,-0.11733908206224442,-0.14398877322673798,0.3103761374950409,-0.0728558599948883,0.3394389748573303,0.24665036797523499,-0.09934282302856445,0.01182454451918602,0.008053685538470745,-0.07300550490617752,-0.16115741431713104,0.12144715338945389,0.01264221966266632,-0.2869531214237213,-0.17859674990177155,0.38765066862106323,0.37601104378700256,-0.02204759046435356},{0.49831315875053406,0.09722068160772324,-0.0935831144452095,-0.12531210482120514,-0.15959985554218292,-0.3156875669956207,-0.31516265869140625,-0.08130313456058502,0.24060125648975372,0.1840069591999054,-0.11798910796642303,0.19312526285648346,-0.11967459321022034,0.3198741674423218,-0.0060549345798790455,-0.2783336341381073,-0.24310734868049622,-0.0636034831404686,0.11387183517217636,-0.18163828551769257,-0.13788475096225739,0.10724452883005142,-0.4400992691516876,-0.23450449109077454,-0.2173628807067871,-0.25182753801345825,-0.18306225538253784,-0.10895027965307236,0.19680729508399963,-0.12718991935253143,-0.03214370086789131,-0.10971710085868835},{-0.061043575406074524,-0.15171219408512115,0.019466429948806763,0.06278965622186661,0.10846378654241562,-0.12995286285877228,-0.0736047700047493,-0.0442064106464386,0.2622379660606384,-0.3644149601459503,0.2449842244386673,0.09892743825912476,-0.09617871046066284,0.32835227251052856,-0.2876729369163513,0.14335240423679352,0.3039129078388214,0.05196348950266838,0.04135437682271004,0.22234657406806946,0.09242866933345795,0.10288619995117188,-0.1947547197341919,-0.07036793231964111,-0.1571754813194275,0.027302702888846397,-0.2891276180744171,-0.30075791478157043,-0.3519115746021271,0.05431248992681503,0.17128096520900726,-0.006292794365435839},{0.21783575415611267,0.26381179690361023,0.2769032418727875,0.1937759667634964,0.3042318820953369,0.30963054299354553,0.34075409173965454,0.3382800221443176,-0.3848358690738678,0.18462194502353668,-0.0797324851155281,0.2179311215877533,0.273296594619751,0.07669977843761444,-0.2700917422771454,-0.20284155011177063,0.067621149122715,0.27629753947257996,0.03588081896305084,-0.2973325550556183,-0.21051225066184998,-0.04922480136156082,0.14753271639347076,0.27250605821609497,-0.2908191680908203,0.14558327198028564,-0.34017670154571533,-0.1485951989889145,0.03570277616381645,-0.2830577790737152,0.2793101370334625,-0.12426981329917908},{-0.10571374744176865,-0.2609175741672516,0.1444588452577591,0.023158973082900047,0.35603198409080505,0.1279921978712082,0.2458089143037796,-0.4565868377685547,0.2798428237438202,0.2714349031448364,-0.28180575370788574,0.2646583318710327,-0.19233892858028412,0.1486525982618332,0.043811481446027756,-0.2954650819301605,0.25580161809921265,-0.09657516330480576,-0.030834782868623734,0.056827329099178314,-0.27547457814216614,-0.11026808619499207,-0.2707013785839081,0.2172679752111435,-0.16594834625720978,-0.23359251022338867,0.33862215280532837,0.054476525634527206,0.32708117365837097,0.2601458430290222,-0.04181371629238129,0.1337573081254959},{-0.008086184971034527,-0.2172359824180603,-0.09695928543806076,-0.17075850069522858,-0.1322302669286728,-0.18118654191493988,-0.0799008309841156,0.29701340198516846,0.3320578336715698,-0.18490229547023773,0.09334981441497803,0.23452027142047882,0.20061135292053223,0.2491883635520935,-0.37689006328582764,0.002405466279014945,0.26398035883903503,-0.23869070410728455,0.15471601486206055,-0.22546452283859253,0.2863360643386841,0.03541276603937149,0.17415887117385864,0.03659974783658981,-0.21115975081920624,0.19754678010940552,0.05856023356318474,-0.204622283577919,0.22919145226478577,0.29116228222846985,0.26645776629447937,-0.01726531609892845},{0.22989560663700104,0.27111828327178955,0.028121456503868103,-0.012342712841928005,-0.27774807810783386,0.008285735733807087,-0.062452323734760284,0.07016653567552567,-0.005580700933933258,0.2066708505153656,0.25487393140792847,-0.16650605201721191,0.2201111912727356,-0.08514279127120972,-0.22482873499393463,-0.19399060308933258,-0.18808287382125854,-0.21276405453681946,-0.09255412220954895,0.17165438830852509,-0.2995167672634125,0.0013403674820438027,0.19530373811721802,-0.21388451755046844,-0.14881326258182526,-0.26080071926116943,-0.08350531756877899,0.155156210064888,-0.1902189701795578,0.1639827936887741,-0.23237265646457672,-0.16712918877601624},{-0.09179747104644775,0.0615534633398056,0.22314676642417908,-0.014629284851253033,0.2716187536716461,0.11766868084669113,-0.10266868025064468,0.06034288927912712,-0.2793634831905365,-0.3851014971733093,-0.15526074171066284,0.15735465288162231,-0.1507396250963211,-0.20285123586654663,-0.23775739967823029,0.1448473334312439,-0.11818794906139374,0.19829483330249786,-0.031271014362573624,0.17833353579044342,0.23846997320652008,0.15445438027381897,-0.19235706329345703,0.023041799664497375,-0.3402255177497864,-0.2214256376028061,-0.12699191272258759,0.11751195043325424,0.16501888632774353,-0.003007258288562298,0.15159054100513458,0.029784070327878},{-0.12904585897922516,-0.26719245314598083,0.09390877932310104,-0.32441022992134094,-0.15058624744415283,0.25705575942993164,-0.10146430879831314,-0.05547190457582474,-0.2911374568939209,-0.04255390912294388,0.20659761130809784,0.16862840950489044,-0.11337677389383316,0.29255446791648865,0.12996427714824677,0.22840707004070282,0.27449530363082886,-0.12840422987937927,0.32648274302482605,0.2920478582382202,0.1647959202528,0.28861960768699646,0.0409439317882061,-0.29704809188842773,-0.0836239755153656,0.10166222602128983,-0.11264541000127792,0.0023249804507941008,-0.0872248038649559,0.11339163780212402,0.04020839184522629,-0.16864025592803955},{0.21049530804157257,-0.01924370601773262,-0.09213133901357651,-0.13106457889080048,-0.17302820086479187,-0.16520178318023682,-0.011795016005635262,-0.2041475623846054,-0.05619870126247406,-0.31306353211402893,-0.1475525200366974,-0.04363967478275299,-0.09192252904176712,-0.2388457953929901,0.28057464957237244,-0.43067511916160583,-0.14039213955402374,-0.02502591535449028,-0.0719980001449585,-0.1561078429222107,0.11736255139112473,0.11550267785787582,-0.1781754046678543,-0.22667138278484344,0.4403350055217743,0.3095013499259949,-0.11100386083126068,0.13105563819408417,0.04921770095825195,0.20154675841331482,-0.09867609292268753,0.38350123167037964},{-0.15843884646892548,-0.22110800445079803,-0.02867812104523182,-0.0221429243683815,-0.13408540189266205,-0.09555606544017792,-0.10532893240451813,-0.4490682780742645,0.07498172670602798,0.20497967302799225,-0.2028915286064148,-0.22943903505802155,-0.2706180214881897,-0.16019365191459656,0.3045060932636261,0.25371694564819336,-0.22837360203266144,0.05866209790110588,-0.08463546633720398,-0.13409094512462616,-0.12182805687189102,0.02104785479605198,-0.12755487859249115,0.1813867688179016,-0.01190771721303463,-0.2997667193412781,-0.061890315264463425,-0.10176894068717957,-0.1755852997303009,-0.2360018491744995,0.22266548871994019,0.009256094694137573},{0.17511123418807983,0.30315274000167847,-0.024796077981591225,-0.4023120105266571,-0.2902456521987915,-0.1419372409582138,-0.3350123465061188,0.22619971632957458,-0.10646034777164459,0.040772292762994766,-0.18253447115421295,0.20188269019126892,-0.26322805881500244,-0.09375006705522537,0.23877829313278198,-0.30684781074523926,-0.2287045568227768,0.04053483530879021,-0.04110702872276306,0.044716935604810715,-0.11014789342880249,-0.035883475095033646,-0.07531244307756424,-0.2631032168865204,0.26113051176071167,-0.01614433526992798,0.038827359676361084,-0.1979459673166275,0.2691040337085724,0.3081111013889313,-0.34354081749916077,0.17529410123825073}};
static const float action_parameterization_distribution_linear_weight[32][4] = {{0.2824772298336029,0.32640188932418823,-0.07970714569091797,0.34686747193336487},{-0.30207616090774536,0.3372974693775177,0.044471919536590576,-0.05681534484028816},{0.20460644364356995,0.17994152009487152,0.18539223074913025,-0.2414407879114151},{-0.2576671838760376,-0.3425140678882599,0.27580511569976807,-0.11494673043489456},{-0.15470270812511444,0.004449129570275545,-0.20023183524608612,-0.37911224365234375},{-0.28214484453201294,0.04452072083950043,0.047120895236730576,-0.28892266750335693},{-0.2780863046646118,-0.04658878222107887,0.1291157454252243,-0.15588043630123138},{-0.22571317851543427,-0.30114856362342834,0.30390363931655884,0.24570158123970032},{0.1520220786333084,0.17401307821273804,-0.2468627691268921,-0.2714459002017975},{-0.010451286099851131,0.15479053556919098,0.16683481633663177,0.03729772940278053},{-0.27904248237609863,-0.265752911567688,-0.24557222425937653,0.27031365036964417},{-0.20422111451625824,-0.04321691766381264,-0.39671117067337036,0.038349539041519165},{0.1109069362282753,-0.1335500180721283,-0.2534720003604889,-0.2784985303878784},{0.4023919105529785,0.13480478525161743,-0.023485330864787102,0.33985450863838196},{0.35305842757225037,0.08259101212024689,0.29345041513442993,-0.2245788276195526},{-0.24598993360996246,-0.1675276905298233,-0.16647440195083618,-0.13078798353672028},{0.17458336055278778,-0.17654189467430115,-0.37338247895240784,-0.02875482849776745},{0.33722954988479614,0.30269598960876465,0.21606996655464172,-0.26146426796913147},{-0.11283080279827118,0.0011956840753555298,0.012378420680761337,0.15502823889255524},{0.21074941754341125,0.29118025302886963,-0.1644763946533203,-0.3694840371608734},{-0.06629741191864014,0.16120260953903198,-0.2846050560474396,-0.18526405096054077},{-0.05671429634094238,0.17457826435565948,0.03359472006559372,-0.009389824233949184},{0.10864222049713135,-0.35352110862731934,-0.23420269787311554,-0.1849171668291092},{0.23779058456420898,-0.2122228592634201,-0.1143135353922844,0.11451533436775208},{0.01087176613509655,-0.04502226039767265,0.2966667413711548,0.2941218912601471},{-0.05183883011341095,-0.14290151000022888,0.33699625730514526,-0.21218331158161163},{0.31324928998947144,-0.2689119279384613,0.04121210426092148,0.2496652603149414},{0.24904944002628326,-0.3003177046775818,0.343362033367157,0.06268066167831421},{-0.25851351022720337,0.17263245582580566,-0.02995484694838524,-0.04022999852895737},{-0.08755812793970108,-0.2934226393699646,-0.2586067318916321,0.06216028332710266},{-0.4168350398540497,-0.004630375187844038,0.16049093008041382,-0.012470738962292671},{0.051756467670202255,-0.33475175499916077,-0.12356521934270859,-0.044024087488651276}};
static const float actor_encoder_self_encoder_0_bias[16] = {-0.11157646030187607,0.045545730739831924,-0.11821288615465164,0.10227424651384354,-0.0329243429005146,0.030978767201304436,-0.0732942521572113,0.021704696118831635,-0.08859141170978546,-0.01445799320936203,-0.06067051738500595,0.08364565670490265,0.03419703245162964,0.06982758641242981,0.037759121507406235,-0.00570708978921175};
static const float actor_encoder_self_encoder_2_bias[16] = {0.0373874306678772,-0.014048421755433083,-0.057436708360910416,-0.00130340619944036,0.137225940823555,-0.032041531056165695,-0.061977483332157135,0.07707441598176956,0.05704880878329277,-0.015567397698760033,-0.07775882631540298,-0.014474290423095226,-0.032524485141038895,0.0013433373533189297,-0.054800137877464294,-0.00545957125723362};
static const float actor_encoder_feed_forward_0_bias[32] = {-0.0010586066637188196,-0.03059689886868,0.022853393107652664,-0.09983919560909271,-0.030752768740057945,-0.04454171657562256,-0.02359590493142605,0.04587145522236824,0.011684281751513481,0.030733073130249977,-0.02746027521789074,-0.02126973867416382,-0.016716621816158295,0.052946366369724274,0.022018760442733765,-0.03212694078683853,0.0022369117941707373,0.039441175758838654,0.011936926282942295,0.009618978016078472,-0.0004433062858879566,-0.004785880912095308,0.0001546627754578367,0.005120699293911457,0.01387830637395382,0.011697045527398586,0.0315229557454586,0.019546302035450935,-0.02288006618618965,-0.03668619692325592,-0.045253653079271317,-0.0031029845122247934};
static const float action_parameterization_distribution_linear_bias[4] = {0.0366840697824955,0.014312121085822582,0.009314475581049919,0.013576115481555462};
static const float actor_encoder_obstacle_encoder_0_weight[16][4] = {{0.16088172793388367,0.23299166560173035,-0.3623979687690735,-0.22422254085540771},{-0.20669855177402496,-0.1454077959060669,-0.22692003846168518,0.23201365768909454},{-0.07818020135164261,0.17966333031654358,-0.32906630635261536,-0.08285078406333923},{0.25198206305503845,0.017634836956858635,0.1722237765789032,-0.48511263728141785},{-0.46430376172065735,-0.4144660532474518,0.21775192022323608,-0.5825053453445435},{0.1008453518152237,-0.08184461295604706,-0.39141330122947693,-0.14975711703300476},{-0.13018342852592468,-0.33535730838775635,-0.3427113890647888,0.24399103224277496},{-0.08544880896806717,-0.006851469166576862,-0.3463595509529114,-0.40438568592071533},{-0.27808108925819397,-0.37732210755348206,-0.2953064441680908,0.47404637932777405},{-0.481741338968277,-0.017055680975317955,0.3667793869972229,-0.05696163699030876},{0.21764029562473297,0.03351730853319168,-0.35313159227371216,-0.44156262278556824},{-0.5627467632293701,-0.5190297365188599,-0.30039238929748535,-0.031223244965076447},{-0.4268482029438019,0.35109230875968933,-0.6894879341125488,0.029414720833301544},{0.3495943546295166,-0.5014052391052246,-0.6270585060119629,0.10532727092504501},{0.4847806692123413,-0.20097288489341736,0.3464074730873108,0.26734524965286255},{-0.0145570682361722,0.3841264843940735,-0.5207710862159729,0.228670135140419}};
static const float actor_encoder_obstacle_encoder_2_weight[4][4] = {{0.31277504563331604,-0.5145304203033447,0.31989291310310364,-0.08873659372329712},{-0.5806160569190979,0.7129580974578857,0.21693666279315948,-0.5537784695625305},{-0.5765459537506104,-0.7045289278030396,-0.1628522127866745,-0.46201008558273315},{-0.011026597581803799,0.5433023571968079,-0.15186677873134613,-0.6208962202072144}};
static const float actor_encoder_obstacle_encoder_0_bias[4] = {-0.02859356813132763,-0.06796471774578094,-0.03179217502474785,-0.08427777141332626};
static const float actor_encoder_obstacle_encoder_2_bias[4] = {-0.05885680019855499,-0.011858680285513401,0.042070601135492325,0.08153975754976273};
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



int main(const float *self_indatav, float *obst_indatav, float *obst_outdata, float *outdatav)
{
    size_t i;
    control_t_n motorThrusts;

    obstacleEmbedder(obst_indatav);

    for (int i = 0; i < D_MODEL; i++) {
        obst_outdata[i] = obstacle_embeds[i];
    }

    networkEvaluate(&motorThrusts, self_indatav);

    outdatav[0] = motorThrusts.thrust_0;
    outdatav[1] = motorThrusts.thrust_1;
    outdatav[2] = motorThrusts.thrust_2;
    outdatav[3] = motorThrusts.thrust_3;

    return EXIT_SUCCESS; 
}
