
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
static const float actor_encoder_self_encoder_0_weight[18][16] = {{-0.6245002150535583,0.42659249901771545,-0.20837652683258057,0.016817841678857803,0.001708011724986136,-0.07455148547887802,0.17634126543998718,-0.13690981268882751,0.017963707447052002,0.12319768220186234,-0.06983587890863419,0.0613153912127018,-0.14208285510540009,0.10041666030883789,-0.27300989627838135,0.07279524952173233},{-0.45645809173583984,-0.07910266518592834,-0.015401368029415607,0.17579440772533417,-0.03350177779793739,0.4764990508556366,0.020093757659196854,0.12114541232585907,0.09323670715093613,-0.32906079292297363,0.012037748470902443,0.5308962464332581,0.22536100447177887,-0.25223416090011597,0.1336980015039444,-0.06605319678783417},{-0.06735405325889587,-0.08005702495574951,-0.2725702226161957,-0.25750869512557983,0.3133063018321991,-0.17110279202461243,-0.8268263339996338,-0.06118696182966232,0.21929503977298737,0.4882951080799103,-0.20791228115558624,0.12022192776203156,-0.2274666279554367,0.42880627512931824,0.17347267270088196,-0.16533209383487701},{-0.35740557312965393,0.3848959803581238,-0.14290118217468262,0.03498448058962822,0.13796767592430115,0.1487835943698883,0.015290684998035431,-0.049338314682245255,0.1774815320968628,0.04690089076757431,-0.05397313833236694,0.004736753646284342,-0.1253945678472519,-0.23716343939304352,-0.13394832611083984,0.10081633925437927},{-0.13410402834415436,-0.032673180103302,-0.23810996115207672,0.10070259124040604,-0.3085736334323883,0.04161108657717705,-0.05730225518345833,-0.07116027921438217,0.024596979841589928,-0.0269277635961771,0.0817931666970253,0.4005222022533417,-0.20915962755680084,-0.08042381703853607,0.05456680431962013,0.16862231492996216},{-0.013728192076086998,0.12892335653305054,-0.31375446915626526,-0.27226316928863525,0.2143198400735855,-0.06150728464126587,-0.4278861880302429,0.10021454840898514,0.4425429105758667,-0.0905332863330841,-0.1467878222465515,-0.025037512183189392,-0.1949879229068756,0.3140203654766083,0.04538226127624512,-0.09393931925296783},{-0.025113539770245552,-0.11006280034780502,-0.4342553913593292,0.4269774258136749,-0.11168159544467926,-0.2805795669555664,-0.10565271228551865,-0.22357724606990814,-0.41300201416015625,-0.3402254581451416,-0.304708331823349,-0.08386486768722534,0.12941749393939972,0.2055896818637848,-0.07795701920986176,-0.07163897156715393},{0.1584772765636444,0.22920064628124237,-0.49428877234458923,-0.06565515697002411,-0.2355671226978302,0.3802739083766937,-0.49444103240966797,0.5992906093597412,-0.3712555170059204,0.4730522036552429,0.16289888322353363,-0.12644827365875244,0.057771649211645126,-0.10618125647306442,-0.2862088680267334,0.6806796789169312},{-0.5542163848876953,0.09403147548437119,-0.1663467288017273,0.9765352010726929,0.4452897906303406,0.24702037870883942,-0.07428654283285141,0.32247602939605713,0.4868778586387634,0.32647261023521423,-0.5459151864051819,-0.2944750189781189,0.42403748631477356,-0.09226087480783463,0.28407955169677734,0.1639494150876999},{-0.04661516472697258,0.25393933057785034,0.24421963095664978,-0.06429452449083328,0.0354590117931366,-0.31078198552131653,0.3273700475692749,-0.3068174123764038,-0.04214591160416603,-0.38560834527015686,-0.265245258808136,0.41966113448143005,0.03769615665078163,0.3197902739048004,-0.047707751393318176,-0.772611677646637},{0.3422148525714874,-0.2303604781627655,-0.7129427790641785,0.5214377641677856,0.27710944414138794,-0.023164663463830948,0.4334346652030945,-0.32345524430274963,-0.36850741505622864,0.009010151959955692,-0.3020919859409332,-0.0044298009015619755,-0.27940741181373596,-0.46194878220558167,0.0967191532254219,0.3416162431240082},{-0.4738559424877167,-0.18857325613498688,-0.23963767290115356,-0.5840425491333008,0.00295909377746284,0.4754860997200012,0.10842902213335037,-0.6386402249336243,-0.6596765518188477,-0.08863820880651474,-0.4350881278514862,0.47074180841445923,-0.07546503841876984,0.02831140346825123,0.880091667175293,0.30799323320388794},{-0.46880269050598145,0.3216652572154999,0.27328649163246155,0.48538994789123535,-0.0447392575442791,0.22346201539039612,-0.011335122399032116,0.10243917256593704,0.11281285434961319,-0.7367902398109436,-0.2861696779727936,-0.37616243958473206,0.5812597274780273,-0.2574242949485779,-0.08954884111881256,0.3051837384700775},{0.29948335886001587,0.2619168758392334,-0.16281917691230774,0.19589577615261078,-0.049097608774900436,-0.11005998402833939,0.02324482426047325,-0.3925630450248718,-0.27223414182662964,-0.48597466945648193,-0.162374809384346,0.2995629608631134,-0.16418619453907013,0.23827455937862396,0.21349649131298065,0.0914468988776207},{-0.3148418366909027,-0.1872190684080124,-0.12079514563083649,-0.2817392945289612,-0.10582002252340317,-0.48317065834999084,0.12146381288766861,0.47046804428100586,-0.4447082579135895,0.37886059284210205,-0.18392597138881683,-0.051936205476522446,-0.20158839225769043,0.5210171937942505,0.2966245710849762,0.2167925238609314},{-0.015973849222064018,0.053827185183763504,-0.2558901906013489,-0.024065639823675156,-0.19031736254692078,-0.18092022836208344,-0.1515321433544159,-0.2082752287387848,-0.17023278772830963,-0.026926739141345024,0.07672935724258423,0.2984626889228821,0.16554051637649536,0.2965087294578552,0.0516873374581337,0.1430395245552063},{0.12507471442222595,-0.2880268394947052,0.1236414834856987,-0.18187260627746582,-0.09183664619922638,0.2814759314060211,0.15757931768894196,0.020474018529057503,-0.28589755296707153,0.2845252454280853,0.10609962046146393,-0.11389782279729843,-0.34414076805114746,0.23600924015045166,0.16766414046287537,-0.052550267428159714},{0.2559868097305298,0.04715227708220482,0.050203025341033936,0.016886217519640923,0.05179661512374878,0.15888388454914093,-0.09553621709346771,0.059303879737854004,-0.22489117085933685,0.4351334571838379,0.20760172605514526,0.14067456126213074,-0.43282967805862427,-0.2709248661994934,0.0749850794672966,0.32866042852401733}};
static const float actor_encoder_self_encoder_2_weight[16][16] = {{0.10541234165430069,0.1566949039697647,-0.06551094353199005,-0.08870323747396469,-0.15213558077812195,-0.2233228087425232,0.5124068856239319,-0.2380814403295517,-0.32467302680015564,-0.1284693330526352,-0.05289052799344063,-0.2582980692386627,-0.14213445782661438,0.31003373861312866,-0.16840580105781555,0.18490518629550934},{-0.3056134581565857,0.11652981489896774,0.149856835603714,-0.020188862457871437,-0.27949395775794983,-0.1178722083568573,-0.3511909246444702,0.3689378798007965,0.37275567650794983,-0.09343262016773224,0.13028892874717712,0.3283247947692871,0.15978942811489105,0.46841827034950256,-0.3478209376335144,-0.41813334822654724},{0.2675057649612427,0.18321822583675385,0.43625104427337646,-0.04923403263092041,-0.09432119876146317,-0.18365991115570068,-0.013114768080413342,-0.30393677949905396,0.49622824788093567,0.28037938475608826,0.4009205996990204,-0.33864718675613403,0.038414809852838516,-0.37429079413414,-0.1722802072763443,0.30211642384529114},{-0.16590116918087006,0.03973632678389549,0.018713321536779404,0.24418149888515472,-0.22031214833259583,0.4066453278064728,0.19747929275035858,0.40881699323654175,-0.41066551208496094,0.09387471526861191,-0.14095094799995422,0.05085437744855881,0.4614509344100952,0.24679690599441528,0.2745267152786255,0.39037418365478516},{-0.12157988548278809,0.29005858302116394,-0.07081596553325653,0.4045315682888031,0.155210942029953,0.31955572962760925,0.08441285043954849,-0.4993310868740082,-0.05298847332596779,-0.04638027399778366,0.07228542864322662,0.22805717587471008,0.06450167298316956,0.13424980640411377,0.07610449939966202,0.07598619163036346},{0.2205430567264557,-0.09174449741840363,-0.28079259395599365,-0.37848636507987976,-0.30903640389442444,0.08534449338912964,0.17274890840053558,0.034690503031015396,0.0540640652179718,-0.32323911786079407,-0.2722536027431488,0.3631623089313507,0.039396245032548904,0.27952903509140015,0.23858556151390076,-0.2988418936729431},{-0.2993612587451935,0.44300979375839233,-0.12478025257587433,0.010954305529594421,-0.050240203738212585,0.35482195019721985,-0.29129758477211,0.27172723412513733,0.023815371096134186,0.46897420287132263,-0.19646351039409637,0.34284159541130066,-0.16491444408893585,0.3618668019771576,0.2807430326938629,0.2774830162525177},{-0.008425637148320675,-0.29594823718070984,-0.2727165222167969,0.47842514514923096,-0.3857966363430023,0.4604714810848236,-0.4036721885204315,0.2852175831794739,-0.276913583278656,0.053201399743556976,-0.0707532986998558,-0.424667626619339,-0.34421753883361816,0.3121376037597656,0.052902109920978546,0.25559085607528687},{0.2400219589471817,-0.2482726275920868,0.35904350876808167,-0.1642731875181198,0.4114335775375366,-0.345140665769577,-0.24274377524852753,0.2325494885444641,0.29092708230018616,0.20208439230918884,0.14847980439662933,-0.307448148727417,0.05513518676161766,0.163008451461792,-0.030747422948479652,-0.3178526759147644},{0.02808236889541149,-0.28798919916152954,0.3736957609653473,0.3908390700817108,-0.17303648591041565,-0.22339476644992828,-0.3830782175064087,0.24937979876995087,-0.17707717418670654,0.06598585098981857,0.08906713128089905,-0.4054132401943207,-0.12362045049667358,-0.3056861460208893,-0.2732565701007843,-0.024033283814787865},{0.26087716221809387,-0.25170281529426575,0.010519697330892086,-0.4094170331954956,-0.40170571208000183,-0.03422176092863083,-0.39691001176834106,0.37834271788597107,-0.14099732041358948,0.18118155002593994,0.17330190539360046,-0.29882875084877014,-0.4592066705226898,0.10167617350816727,0.2471737414598465,-0.03451424092054367},{-0.43617111444473267,-0.13100983202457428,-0.07530884444713593,-0.19431567192077637,0.080307736992836,-0.2535412609577179,-0.18964965641498566,0.32448795437812805,-0.048076435923576355,-0.15685102343559265,-0.2477523386478424,-0.30493128299713135,-0.16401340067386627,0.23609639704227448,0.08177607506513596,-0.2522766888141632},{0.26752418279647827,-0.3153134882450104,-0.08450254797935486,-0.007773911580443382,-0.27161791920661926,0.1349174529314041,-0.03176816552877426,0.2789914608001709,0.24556827545166016,0.08940953016281128,-0.1082693412899971,-0.19271047413349152,0.03421228751540184,0.16904671490192413,-0.3319411873817444,0.35735857486724854},{-0.3298359513282776,0.0935903936624527,0.34505996108055115,0.2752721309661865,-0.18289704620838165,-0.46371808648109436,-0.2600337266921997,-0.34448739886283875,0.1932438760995865,0.09350020438432693,-0.1547926515340805,0.17070703208446503,0.02003103494644165,-0.06840478628873825,0.07499554008245468,0.09827379882335663},{-0.005387553945183754,-0.4474990665912628,-0.4248599410057068,0.3907988667488098,0.19225570559501648,-0.14523334801197052,-0.18219007551670074,0.29768452048301697,0.5485667586326599,0.2796519100666046,-0.3444887101650238,0.05789029970765114,0.04156709462404251,-0.02246357500553131,-0.4019518494606018,-0.29538694024086},{0.2041332721710205,0.43249788880348206,-0.1588553488254547,0.36676692962646484,-0.014225360937416553,0.2873317301273346,0.5595499277114868,0.08963632583618164,0.2694019377231598,-0.14444485306739807,-0.4206482470035553,-0.08323309570550919,-0.1349494457244873,-0.19011759757995605,-0.5136297941207886,-0.3023209273815155}};
static const float actor_encoder_feed_forward_0_weight[20][20] = {{0.10822590440511703,0.07650993764400482,0.16859070956707,0.34304943680763245,-0.010122742503881454,0.35019272565841675,-0.30852875113487244,0.22625012695789337,0.11209090799093246,0.013455204665660858,0.13478431105613708,-0.14651772379875183,0.07372859120368958,-0.21557535231113434,-0.2702710032463074,-0.029880279675126076,-0.2674500048160553,-0.1484658420085907,0.16159653663635254,0.20275214314460754},{0.09726553410291672,0.19559361040592194,0.09134022891521454,0.4874572157859802,-0.3212065100669861,0.36312147974967957,0.40796709060668945,0.3602656126022339,-0.1869109719991684,-0.44513437151908875,-0.3522210419178009,0.0892954170703888,-0.1328992247581482,0.21061912178993225,-0.3609999418258667,-0.19907093048095703,0.11020060628652573,0.470873087644577,-0.12317649275064468,0.24769620597362518},{0.13295571506023407,0.10144349187612534,0.09902537614107132,0.05148156359791756,-0.07565456628799438,-0.0006244819960556924,-0.032822657376527786,-0.2703774869441986,0.47872793674468994,0.10452120006084442,-0.3477761149406433,-0.2564569115638733,0.27631106972694397,-0.055995143949985504,0.15140436589717865,-0.46149033308029175,0.17040833830833435,0.25808846950531006,-0.007112608756870031,-0.3968285918235779},{0.24312041699886322,-0.10955039411783218,-0.0851203128695488,-0.4666883647441864,-0.3363948166370392,-0.4285270869731903,-0.2960521876811981,-0.3082820773124695,-0.08838505297899246,0.23233626782894135,-0.05237143486738205,0.02505812793970108,0.07486703246831894,-0.20090016722679138,-0.09303293377161026,0.175476536154747,-0.21758577227592468,0.060273777693510056,0.3288678228855133,-0.3874809443950653},{0.16217148303985596,0.23974767327308655,-0.15858888626098633,-0.22087348997592926,0.1758497655391693,0.056965623050928116,0.4593881666660309,-0.020658664405345917,0.04369036853313446,0.39608335494995117,0.3434232771396637,-0.06467675417661667,-0.24908079206943512,0.06906306743621826,-0.3776908218860626,0.04432656615972519,0.18352110683918,-0.10526154935359955,0.24395465850830078,0.30947181582450867},{-0.08167847990989685,-0.025559809058904648,-0.0546087883412838,0.34214627742767334,-0.3267742097377777,-0.02167649008333683,0.24564751982688904,-0.10823172330856323,-0.3243120610713959,-0.480327308177948,-0.30267536640167236,0.4453718960285187,0.06058763340115547,-0.32256168127059937,-0.011739655397832394,-0.1581539511680603,0.45895472168922424,0.15905863046646118,-0.050190333276987076,-0.03906368091702461},{0.27004581689834595,-0.3079233467578888,0.3444402515888214,0.2549384534358978,0.2681627869606018,0.21982714533805847,-0.21975849568843842,-0.3562992215156555,-0.08992289751768112,0.04708174616098404,-0.08766275644302368,0.2623648941516876,-0.06370946019887924,-0.04914364963769913,-0.4364541471004486,-0.26851391792297363,0.20741228759288788,0.12560716271400452,-0.37180715799331665,0.1409623771905899},{0.257914662361145,-0.200231671333313,0.4578597843647003,0.31883862614631653,0.35077112913131714,-0.30208250880241394,-0.08060332387685776,0.03641762584447861,0.00998213142156601,0.16627748310565948,-0.48576119542121887,0.3219774663448334,0.07804948091506958,-0.13868871331214905,-0.031131185591220856,0.04277056083083153,0.003204948967322707,0.03377193585038185,0.02736213617026806,-0.3556210696697235},{-0.27788662910461426,-0.17716717720031738,-0.029928015545010567,0.013114100322127342,-0.22196824848651886,0.42771005630493164,-0.3613145649433136,0.15982136130332947,0.5197608470916748,0.0896305963397026,-0.2867542803287506,0.3165351450443268,-0.1050836518406868,0.2243882119655609,-0.04153412953019142,-0.5776036977767944,0.11310942471027374,-0.38979968428611755,-0.1927383840084076,-0.2405208945274353},{-0.4257771372795105,0.1384442150592804,0.11440648883581161,-0.17448922991752625,0.26497748494148254,0.3702707886695862,-0.054519131779670715,0.22873421013355255,0.3398374915122986,-0.0136504415422678,-0.06199910119175911,0.09808443486690521,0.02704842947423458,-0.36951372027397156,-0.24718397855758667,-0.12898185849189758,0.3411174416542053,0.3855692446231842,0.3267449140548706,0.3131222128868103},{0.08051279932260513,-0.0641988217830658,0.012997073121368885,0.422192245721817,0.33928510546684265,0.06487689912319183,0.37379372119903564,-0.28464585542678833,0.15596023201942444,0.08184057474136353,-0.19714860618114471,-0.27959364652633667,0.3382577896118164,-0.43407943844795227,-0.13267932832241058,0.01222318410873413,0.02400313690304756,0.020855197682976723,-0.2201746702194214,-0.07954144477844238},{-0.12847600877285004,-0.30526769161224365,-0.08974814414978027,0.2537045478820801,-0.3023183345794678,0.027864200994372368,-0.08163061738014221,-0.11816079169511795,0.335773766040802,0.2390352189540863,-0.014017816632986069,-0.12106507271528244,-0.2482723593711853,0.24050846695899963,-0.25747084617614746,0.2632916271686554,-0.29398325085639954,0.19404126703739166,0.2774312198162079,0.5471690893173218},{0.0596076138317585,-0.2453155815601349,0.03951884061098099,-0.05650453269481659,-0.19318373501300812,-0.2503010928630829,0.4942518174648285,-0.2903251349925995,-0.2667122483253479,0.05431656539440155,0.08155971765518188,-0.23489151895046234,-0.005009100306779146,-0.2616283893585205,-0.40951916575431824,-0.36907652020454407,0.3715019226074219,0.24398371577262878,0.41126683354377747,-0.03977498039603233},{0.3960833251476288,0.4148455858230591,0.33053579926490784,-0.09626105427742004,-0.14983372390270233,0.03214312717318535,0.5020703077316284,0.07881955802440643,0.1563236266374588,-0.298994779586792,0.043624650686979294,0.021514087915420532,-0.4014073312282562,0.06963297724723816,-0.2624136805534363,0.045992784202098846,-0.15419219434261322,0.32367685437202454,0.13970984518527985,-0.2419022023677826},{-0.2814803421497345,0.17578324675559998,0.18996073305606842,0.23349320888519287,0.24676957726478577,0.15715160965919495,0.5999307036399841,0.009476353414356709,0.013158278539776802,0.13121697306632996,-0.2507063150405884,0.22344614565372467,-0.11678995192050934,-0.04322950169444084,-0.04958662763237953,0.2640637159347534,-0.3482673168182373,-0.3064222037792206,0.3415791988372803,-0.2536618113517761},{-0.10600428283214569,0.3121090531349182,-0.25188449025154114,0.21433772146701813,0.16059526801109314,0.18452627956867218,0.5481239557266235,-0.05175415799021721,0.2697390019893646,-0.12894003093242645,0.036362532526254654,0.13855892419815063,-0.30764487385749817,0.3468596339225769,-0.014430280774831772,-0.10129003971815109,-0.1971331685781479,0.3817504644393921,-0.032546188682317734,-0.14752113819122314},{0.46202951669692993,-0.17333590984344482,0.2772510051727295,-0.3134019374847412,0.17079439759254456,0.21671272814273834,-0.1074615865945816,-0.23541028797626495,-0.2585607171058655,0.08841532468795776,0.15376533567905426,0.16277287900447845,0.17094972729682922,-0.1352655291557312,0.2698464095592499,-0.11320418864488602,-0.09584355354309082,-0.08146829903125763,-0.09494558721780777,0.25243210792541504},{0.30207279324531555,0.17667168378829956,0.15239442884922028,0.28932031989097595,-0.3467085659503937,-0.011933743953704834,0.49394237995147705,-0.08458687365055084,-0.031608160585165024,0.017678560689091682,0.19214266538619995,-0.25879353284835815,-0.26124849915504456,0.12395315617322922,0.298481822013855,0.3246270418167114,0.09449663758277893,-0.2745041251182556,-0.06653518974781036,0.09540706872940063},{0.30053219199180603,0.27540627121925354,0.07568677514791489,-0.22630690038204193,-0.09773199260234833,-0.2727879285812378,0.16991117596626282,0.23491422832012177,0.0835021436214447,-0.08297137171030045,0.19902464747428894,-0.06760530173778534,0.07258769124746323,-0.25819122791290283,-0.29822760820388794,0.3087965250015259,-0.2881147861480713,0.04935576766729355,0.011263185180723667,0.026664244011044502},{0.28284892439842224,0.009954586625099182,-0.18279103934764862,-0.07450705021619797,0.37762075662612915,0.13362003862857819,-0.07990210503339767,-0.11536488682031631,-0.1854759007692337,0.09671393036842346,-0.1220366582274437,-0.06896568834781647,0.09148336946964264,0.2325851172208786,-0.2250043898820877,0.07580338418483734,0.22649671137332916,-0.40998899936676025,-0.20803657174110413,0.2754044234752655}};
static const float action_parameterization_distribution_linear_weight[20][4] = {{-0.18372422456741333,-0.19610370695590973,-0.342841774225235,-0.354183167219162},{-0.22757963836193085,-0.23214781284332275,0.2538686692714691,0.028976336121559143},{0.11248047649860382,0.2917919158935547,0.09278778731822968,-0.19683341681957245},{0.30764225125312805,0.19229820370674133,0.26334136724472046,0.32223308086395264},{0.16198691725730896,0.032966531813144684,-0.3651333749294281,0.08204028755426407},{0.14269721508026123,0.01417121384292841,0.05986442044377327,0.10732629895210266},{-0.5189729928970337,-0.07642429322004318,-0.2106129229068756,0.33550769090652466},{0.3291231393814087,-0.16568103432655334,-0.2704090476036072,0.4957710802555084},{-0.2590664327144623,0.5816623568534851,-0.36738866567611694,0.461256742477417},{-0.08464334905147552,0.0288462545722723,-0.3114471137523651,-0.21347548067569733},{0.26533764600753784,-0.37912440299987793,-0.5116027593612671,0.2135593295097351},{0.3352404534816742,0.36858099699020386,-0.06932201981544495,0.008083405904471874},{-0.04818347096443176,-0.43339449167251587,0.28784075379371643,0.08337090909481049},{0.19079747796058655,0.28551608324050903,-0.19109876453876495,-0.17340955138206482},{0.43232375383377075,-0.34441912174224854,-0.48729628324508667,0.096273273229599},{0.10789491981267929,-0.6217225193977356,0.19536685943603516,0.0899759978055954},{0.08971970528364182,0.13452839851379395,0.10944931954145432,-0.06069209426641464},{0.30269864201545715,-0.1735302060842514,0.3286910057067871,0.21309694647789001},{-0.016794810071587563,0.059265755116939545,-0.11181490123271942,0.014643737114965916},{0.3359694480895996,0.3192329704761505,-0.4895189702510834,0.4664348065853119}};
static const float actor_encoder_self_encoder_0_bias[16] = {0.07906919717788696,0.0834314376115799,0.17842136323451996,-0.048326633870601654,0.07324673980474472,-0.03446260094642639,0.0660419687628746,0.13599425554275513,-0.07022310793399811,0.04497087001800537,-0.07623059302568436,0.02776300720870495,0.027210818603634834,0.11200632154941559,0.1522812396287918,-0.037302784621715546};
static const float actor_encoder_self_encoder_2_bias[16] = {-0.10931262373924255,0.004131656140089035,0.028762072324752808,0.034537527710199356,-0.0884222537279129,0.04562418907880783,-0.20788602530956268,-0.05809587612748146,0.13078227639198303,0.09700606018304825,-0.026889441534876823,-0.011822741478681564,0.019554447382688522,-0.05954638868570328,-0.00537829240784049,-0.01461013127118349};
static const float actor_encoder_feed_forward_0_bias[20] = {-0.11065758019685745,-0.02799837663769722,-0.050286032259464264,0.01846683956682682,-0.03976183012127876,0.04426928237080574,-0.10176341235637665,-0.008449116721749306,0.04436423256993294,-0.02798023447394371,0.02859974279999733,-0.02781059220433235,0.05844876170158386,-0.06301204115152359,0.06383350491523743,0.01137358509004116,0.05678654834628105,0.10696498304605484,-0.03660590946674347,-0.07486323267221451};
static const float action_parameterization_distribution_linear_bias[4] = {0.04029787331819534,0.01470855250954628,0.05543451011180878,0.051540277898311615};
static const float actor_encoder_obstacle_encoder_0_weight[16][4] = {{0.07725248485803604,-0.2873660624027252,0.3374760150909424,-0.5530890822410583},{0.1025514155626297,-0.17851568758487701,-0.3705648183822632,0.4114498496055603},{-0.313043475151062,-0.388726145029068,0.2778172492980957,-0.21129700541496277},{0.4185054302215576,0.09153825044631958,-0.2435712069272995,-0.17423288524150848},{0.3978908956050873,0.14549313485622406,-0.26341626048088074,-0.27280598878860474},{0.2067713439464569,-0.3438766598701477,-0.3855830132961273,-0.2668420672416687},{0.3384702205657959,-0.16581910848617554,-0.5211455821990967,-0.12955091893672943},{0.32192304730415344,-0.6061798334121704,-0.5669814944267273,-0.5867031216621399},{0.18544220924377441,-0.0884794071316719,-0.5470377206802368,-0.6600214838981628},{0.4560382068157196,-0.3202652037143707,0.13732530176639557,0.09810252487659454},{-0.4154523015022278,-0.6712133288383484,0.046238984912633896,0.13512085378170013},{0.10912429541349411,0.2790311872959137,-0.13970713317394257,-0.4703408181667328},{0.5570930242538452,-0.09834572672843933,-0.05133531242609024,-0.16730955243110657},{-0.25594979524612427,0.16267503798007965,0.2503452003002167,0.08864377439022064},{0.33892500400543213,0.15766382217407227,-0.3456985354423523,0.27361994981765747},{0.47036659717559814,-0.5820236802101135,0.18288446962833405,-0.366026371717453}};
static const float actor_encoder_obstacle_encoder_2_weight[4][4] = {{-0.7446381449699402,-0.17757552862167358,-0.6331912279129028,0.2347969114780426},{-0.40596985816955566,0.8168907165527344,0.18347394466400146,0.6645526885986328},{-0.48582202196121216,0.16665729880332947,-0.17733383178710938,0.09839026629924774},{0.8188258409500122,0.3667784333229065,0.5577394962310791,0.7579081058502197}};
static const float actor_encoder_obstacle_encoder_0_bias[4] = {0.15692530572414398,-0.23592093586921692,-0.037796929478645325,-0.18903382122516632};
static const float actor_encoder_obstacle_encoder_2_bias[4] = {0.018702801316976547,-0.030533136799931526,0.02373066544532776,-0.11995085328817368};
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
