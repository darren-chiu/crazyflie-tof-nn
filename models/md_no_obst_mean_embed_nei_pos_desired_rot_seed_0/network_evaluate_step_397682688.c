#include "network_evaluate.h"

static const int self_structure [4][2] = {{18, 16},{16, 16},{24, 32},{32, 4}};
static const int nbr_structure [2][2] = {{3, 8},{8, 8}};

static const int D_SELF = self_structure[1][1];
static const int D_NBR = nbr_structure[1][1];

static float neighbor_embeds[D_NBR];
static float output_embeds[D_SELF + D_NBR];

static float nbr_output_0[NEIGHBORS][8];
static float nbr_output_1[NEIGHBORS][8];
static float output_0[16];
static float output_1[16];
static float output_2[32];
static float output_3[4];
static const float actor_encoder_self_encoder_0_weight[18][16] = {{0.1459328979253769,0.2542669475078583,0.15184274315834045,-0.14385169744491577,0.05023430660367012,-0.3117097318172455,-0.020553620532155037,-0.5083245635032654,-0.02392609417438507,-0.20546764135360718,-0.12598183751106262,0.1462230086326599,-0.25607815384864807,0.11709144711494446,0.27489182353019714,-0.16053511202335358},{-0.02856733277440071,-0.01397997047752142,-0.03457554802298546,-0.202899768948555,0.36878323554992676,0.08682673424482346,0.07795895636081696,0.03225875273346901,-0.18174830079078674,0.01998777687549591,-0.20197585225105286,0.07290791720151901,0.23986399173736572,0.013618163764476776,0.06365004181861877,-0.09921985864639282},{-0.25193288922309875,-0.33423513174057007,0.2814759314060211,-0.09531614929437637,-0.14682357013225555,0.3595476746559143,-0.45367807149887085,0.380919873714447,-0.40930691361427307,-0.4304310083389282,-0.28563544154167175,-0.31240662932395935,-0.28486257791519165,0.07335381209850311,-0.3891292214393616,-0.4399993419647217},{-0.2993575632572174,-0.0958942174911499,-0.013645417056977749,0.16321855783462524,0.13940231502056122,-0.013535969890654087,0.2816234230995178,-0.18673741817474365,-0.1661905199289322,0.05431561917066574,-0.3253411054611206,-0.057894445955753326,-0.16485530138015747,0.38779377937316895,-0.25338712334632874,0.03127327933907509},{0.10534247756004333,-0.053394872695207596,0.11447934061288834,-0.31292256712913513,0.3738463222980499,0.43262970447540283,-0.05309511721134186,-0.12658724188804626,0.0021934155374765396,0.33118000626564026,0.0939781442284584,-0.0066678705625236034,0.22156724333763123,-0.0023124548606574535,-0.2180362343788147,-0.27394363284111023},{0.1363421082496643,0.15378780663013458,0.16233345866203308,-0.12350504845380783,-0.17349816858768463,-0.06691863387823105,-0.16520735621452332,0.08242464810609818,-0.17576119303703308,0.19671416282653809,-0.2276136428117752,-0.3065679371356964,-0.09786117821931839,0.034314218908548355,-0.27342304587364197,0.02301104925572872},{-0.37984755635261536,-0.502893328666687,0.06479807943105698,-0.2713106870651245,-0.2824588119983673,-0.0481259860098362,0.29260918498039246,0.27928587794303894,-0.6038026809692383,-0.05691001936793327,-0.12841133773326874,0.09739900380373001,-0.34566986560821533,-0.07193763554096222,0.3138846755027771,0.08520075678825378},{-0.033770669251680374,0.513971209526062,0.34122106432914734,0.23020921647548676,-0.2672412395477295,0.22895562648773193,0.47045156359672546,0.5969780087471008,0.023347776383161545,0.0883304551243782,0.09577036648988724,0.1757989078760147,-0.14035910367965698,-0.4830646812915802,0.1994948536157608,0.4248136281967163},{-0.44063979387283325,-0.05457598716020584,0.16611388325691223,-0.1396723985671997,0.10955174267292023,0.17577813565731049,0.1474490463733673,-0.27647048234939575,-0.2218458503484726,-0.4210900366306305,-0.03110361285507679,0.2753373086452484,0.07560783624649048,0.2440694272518158,0.23158200085163116,-0.18312828242778778},{0.6758751273155212,-0.10706274211406708,-0.6404898166656494,-0.20935343205928802,0.21128025650978088,0.045353636145591736,-0.5957787036895752,-0.7264914512634277,-0.5014216899871826,0.33721813559532166,0.034196872264146805,0.45444899797439575,-0.5375682711601257,0.5252125263214111,-0.6442665457725525,-0.07283942401409149},{0.37455177307128906,-0.2446093112230301,0.05168430134654045,0.37299492955207825,-0.2970542311668396,0.31812939047813416,0.3690251111984253,0.34138673543930054,-0.41887399554252625,-0.3228435814380646,-0.4030744433403015,0.30406779050827026,-0.2421010285615921,-0.2665688097476959,-0.20361848175525665,0.18975690007209778},{0.18869227170944214,-0.21220742166042328,-0.1817798912525177,-0.496174156665802,0.30807560682296753,0.02865404635667801,-0.17644089460372925,-0.37966427206993103,-0.3555600345134735,-0.526341438293457,-0.5322293639183044,-0.3242913484573364,0.0936393141746521,-0.3303811848163605,0.48769524693489075,0.35436174273490906},{0.07764909416437149,-0.5637190937995911,-0.3365790843963623,0.04395236819982529,0.1749843955039978,0.1414029598236084,0.2295357882976532,-0.29241839051246643,0.0925542414188385,-0.47379934787750244,0.10302015393972397,-0.3446815013885498,0.4133191406726837,-0.746491014957428,-0.22598503530025482,-0.08982866257429123},{0.04290429875254631,-0.29100993275642395,-0.4955153465270996,0.5116691589355469,-0.16121338307857513,0.057289671152830124,-0.0002947163302451372,-0.07247809320688248,-0.07980462163686752,-0.2769804000854492,0.16153934597969055,-0.06991857290267944,-0.45221710205078125,0.00963314063847065,0.32682669162750244,-0.5472037196159363},{-0.12922678887844086,-0.027838539332151413,-0.3364931046962738,0.24003997445106506,0.36777740716934204,-0.28671959042549133,0.2943063974380493,0.1285805106163025,-0.25338801741600037,0.09111163020133972,0.23479066789150238,0.1541653424501419,-0.15669693052768707,-0.3903183341026306,0.07224492728710175,0.1844216138124466},{-0.27950721979141235,0.14533661305904388,-0.1625555455684662,0.28532570600509644,-0.01348329707980156,-0.31395071744918823,0.22391387820243835,0.13373054563999176,-0.023926619440317154,-0.12144975364208221,0.005850157700479031,-0.04573593661189079,-0.2623048722743988,-0.15285080671310425,-0.27508074045181274,-0.2498224526643753},{-0.2682167887687683,0.18265171349048615,0.04699306562542915,0.025471685454249382,-0.307624489068985,0.11550814658403397,-0.21040469408035278,0.08480024337768555,-0.21584179997444153,0.09408380836248398,-0.00478557962924242,0.27762681245803833,0.19290541112422943,0.29760217666625977,0.3933681547641754,-0.24178546667099},{0.2041817158460617,-0.32714909315109253,-0.17654898762702942,-0.04605316370725632,-0.3499826192855835,-0.2949571907520294,-0.30292096734046936,-0.22939662635326385,-0.2078256458044052,0.02191184088587761,-0.24792860448360443,0.07435069233179092,0.002854305785149336,0.20147959887981415,-0.08890830725431442,-0.31133103370666504}};
static const float actor_encoder_self_encoder_2_weight[16][16] = {{-0.16452670097351074,0.2574140727519989,0.048001643270254135,0.4576275050640106,0.16206030547618866,-0.08919400721788406,-0.21199600398540497,-0.3464771509170532,-0.04695934057235718,-0.12001481652259827,0.22697556018829346,0.0119343101978302,0.34842586517333984,0.08706049621105194,-0.44232577085494995,-0.38647526502609253},{-0.2549413740634918,0.25014203786849976,-0.23320475220680237,-0.05886131525039673,0.16328059136867523,-0.015662670135498047,-0.4863818883895874,0.11594555526971817,0.3112853169441223,0.1073051393032074,0.005635404027998447,-0.019132426008582115,-0.21806617081165314,-0.08133965730667114,-0.29576045274734497,-0.07001689076423645},{0.511873185634613,0.3902117908000946,-0.3795788884162903,-0.570068359375,0.31590035557746887,-0.30847594141960144,-0.5504530668258667,0.22019079327583313,0.3314167261123657,-0.2910064160823822,0.2009461671113968,-0.33885353803634644,-0.1251206248998642,-0.13756918907165527,0.44003474712371826,0.379867821931839},{0.3216954469680786,-0.12670186161994934,-0.29670166969299316,0.2975287437438965,-0.25942614674568176,0.22106502950191498,0.10167240351438522,-0.0761142373085022,-0.08508943021297455,0.3003601133823395,0.3639339804649353,-0.4559865891933441,0.11430804431438446,-0.2701255977153778,-0.35670197010040283,0.05511561036109924},{0.10037107765674591,0.23795367777347565,-0.013688197359442711,-0.04584173858165741,0.013543874956667423,0.038363490253686905,-0.1527734398841858,0.2383010983467102,-0.20851339399814606,-0.3509826362133026,-0.3238151967525482,0.2521198093891144,0.261191189289093,-0.13593889772891998,0.25616931915283203,0.09618908911943436},{0.4157293736934662,0.09863699972629547,-0.2345152199268341,-0.1171284168958664,0.19506190717220306,0.2157273292541504,-0.15568286180496216,-0.3728024661540985,-0.13343696296215057,0.1644289791584015,-0.08417318761348724,-0.039098385721445084,-0.30105841159820557,0.373773992061615,0.13965125381946564,-0.13111265003681183},{0.25683504343032837,-0.353947252035141,-0.39269590377807617,-0.27851060032844543,-0.01554103847593069,0.27390193939208984,-0.3397561311721802,0.018508795648813248,0.15740986168384552,0.12992584705352783,-0.021419622004032135,0.3570653200149536,0.29917800426483154,-0.1283508539199829,0.2430323362350464,0.3538651168346405},{-0.32143789529800415,0.3331649899482727,-0.1353471279144287,0.4270772933959961,-0.0035607763566076756,0.06830625981092453,0.40893542766571045,0.414310485124588,-0.12029501795768738,-0.2427462935447693,0.3147122859954834,-0.37042567133903503,-0.0705108717083931,-0.1911878138780594,-0.227762371301651,-0.08354692161083221},{0.028787078335881233,-0.30274829268455505,-0.023631038144230843,0.13920338451862335,0.12010712921619415,0.19886893033981323,-0.03170404210686684,0.48354822397232056,0.0755537822842598,0.20365023612976074,-0.25873255729675293,-0.19042223691940308,0.2234835922718048,-0.4557671844959259,0.29051467776298523,0.03609291464090347},{-0.07871755212545395,-0.08651844412088394,-0.17281277477741241,0.2553377151489258,-0.36325258016586304,-0.010054153390228748,0.31322309374809265,0.22845907509326935,0.18999794125556946,-0.1913396716117859,-0.16668091714382172,0.01891917549073696,-0.2074943631887436,0.05111450329422951,-0.42679640650749207,-0.030988624319434166},{-0.11837705224752426,-0.15562158823013306,0.14632870256900787,0.29442161321640015,-0.15196876227855682,-0.21949924528598785,0.312288373708725,-0.1202886551618576,-0.1899043321609497,-0.0011531636118888855,-0.05900849774479866,-0.3235374093055725,-0.36028483510017395,-0.5275813937187195,-0.31409668922424316,-0.045132096856832504},{-0.02344188466668129,-0.5404002666473389,0.29862064123153687,-0.14405594766139984,0.14940376579761505,0.1057976484298706,-0.35742172598838806,-0.30725446343421936,0.21777518093585968,0.15807802975177765,-0.5234037637710571,0.3944600224494934,-0.20735643804073334,-0.10050880908966064,0.21180124580860138,0.1217472180724144},{-0.11351310461759567,0.08550097048282623,-0.33635491132736206,0.1188751831650734,0.1859257072210312,0.03266468644142151,0.3026513159275055,-0.2781205177307129,-0.10836119949817657,-0.08031491935253143,-0.34853821992874146,0.3857830762863159,0.3653815984725952,0.1964658945798874,0.05566364899277687,0.3184592127799988},{-0.34139496088027954,-0.04880289360880852,0.27911487221717834,-0.36658647656440735,0.3915710747241974,-0.520041823387146,-0.11848800629377365,-0.15152230858802795,0.26709556579589844,0.22245632112026215,0.29701852798461914,0.3651431202888489,-0.19993016123771667,-0.23422066867351532,-0.2697116732597351,0.28271180391311646},{-0.09995012730360031,-0.17637938261032104,-0.3281539976596832,0.22232815623283386,-0.04889918118715286,0.47234103083610535,0.23625756800174713,0.31295764446258545,0.1823658049106598,0.000271632190560922,-0.19220751523971558,-0.20933610200881958,0.24851906299591064,0.13682889938354492,-0.010020606219768524,0.14727096259593964},{-0.16812220215797424,0.2634555995464325,-0.07288604974746704,-0.224385067820549,0.05219254642724991,0.012282605282962322,0.14450089633464813,0.22606755793094635,-0.1636389195919037,-0.12759259343147278,-0.19082051515579224,0.266386479139328,0.11598699539899826,-0.20915564894676208,-0.35881271958351135,0.09681277722120285}};
static const float actor_encoder_feed_forward_0_weight[24][32] = {{-0.04328625276684761,0.007342781405895948,0.05723396688699722,0.19273656606674194,0.39730304479599,-0.07652135193347931,-0.14750933647155762,0.024823009967803955,-0.22080056369304657,-0.04837065190076828,0.011909643188118935,0.18589019775390625,0.26797908544540405,-0.09002065658569336,0.23756718635559082,0.0941522866487503,0.22760950028896332,0.03560538962483406,0.008854197338223457,0.020976265892386436,0.301945298910141,-0.19518092274665833,-0.2679535746574402,-0.2616717219352722,-0.14844387769699097,0.11256192624568939,-0.28733187913894653,-0.20687265694141388,0.2202240526676178,-0.10803183168172836,-0.300208181142807,-0.14900819957256317},{0.2390328049659729,0.07750436663627625,0.27867376804351807,0.3301754593849182,0.42632097005844116,-0.03609795868396759,0.14934039115905762,-0.3967847228050232,0.19590666890144348,0.14163871109485626,-0.07867050915956497,-0.2426489293575287,-0.20075885951519012,-0.16644728183746338,-0.3026614785194397,0.3342515528202057,0.19124481081962585,-0.2876833379268646,0.15004925429821014,0.16975456476211548,-0.02192387916147709,0.25909319519996643,-0.2567402720451355,0.1187586784362793,0.047341857105493546,-0.3785126209259033,-0.1702616959810257,0.39673617482185364,-0.23585569858551025,0.03753858432173729,0.2756395936012268,-0.11983078718185425},{0.19928224384784698,-0.03010738454759121,-0.19024673104286194,-0.3289690613746643,-0.3496324419975281,-0.17334440350532532,-0.3334708511829376,0.11686672270298004,-0.41134145855903625,-0.1925121247768402,0.09747892618179321,0.3355795741081238,0.14351098239421844,-0.06828974932432175,-0.21223929524421692,0.06514810025691986,0.2548471987247467,-0.1955012083053589,-0.14222107827663422,0.28962793946266174,-0.25894004106521606,-0.020833199843764305,0.011043475940823555,-0.04850852116942406,-0.0014640563167631626,0.43725383281707764,-0.30200907588005066,0.229324609041214,-0.5275617837905884,-0.1284773200750351,0.013676238246262074,0.033300645649433136},{0.2043481171131134,-0.34975960850715637,-0.08994828909635544,0.18790748715400696,0.145344540476799,-0.19784881174564362,0.27683335542678833,0.09455989301204681,0.13000836968421936,-0.048597611486911774,0.05130702257156372,-0.07976895570755005,0.3322942852973938,0.19682322442531586,-0.1687043309211731,-0.17829583585262299,-0.1525781899690628,0.17100593447685242,0.190775528550148,-0.017881346866488457,0.01610603742301464,0.36686035990715027,-0.07146478444337845,-0.20172670483589172,0.0790548026561737,0.1244790181517601,0.1359773725271225,0.07134809345006943,-0.37380337715148926,-0.3496381938457489,0.0929754301905632,0.19114498794078827},{-0.061072852462530136,0.19990891218185425,0.025677718222141266,0.0786222293972969,0.0935627743601799,0.2813202738761902,0.22722740471363068,-0.18141238391399384,-0.05748276039958,0.08257052302360535,-0.15102532505989075,0.07903659343719482,-0.2863832712173462,0.1922362595796585,0.021504489704966545,0.2806685268878937,0.07824632525444031,0.011253812350332737,0.03049595095217228,0.09977342933416367,0.1400257796049118,-0.1067366674542427,-0.285629540681839,-0.24975106120109558,0.0012222619261592627,0.13014747202396393,0.13253897428512573,0.028843645006418228,0.21485063433647156,-0.1560395359992981,-0.16929638385772705,0.26739978790283203},{-0.3002920150756836,0.22067657113075256,-0.023459894582629204,0.00488306162878871,-0.28608009219169617,-0.2953883409500122,-0.133339986205101,0.08740698546171188,0.2596604824066162,-0.16323818266391754,0.0830337405204773,-0.3745996654033661,-0.12839646637439728,0.04294270649552345,0.07892157137393951,-0.14317600429058075,-0.17498114705085754,0.07078596949577332,0.11706466972827911,-0.3713743984699249,0.2061641663312912,-0.01995472051203251,-0.002970220288261771,-0.16955982148647308,0.1791919767856598,0.057325467467308044,0.014737657271325588,0.08916371315717697,0.12323065847158432,-0.07211402803659439,-0.3297939896583557,-0.2388244867324829},{0.03125477954745293,-0.09537728130817413,0.0855603888630867,-0.18469388782978058,-0.1804656982421875,0.08868783712387085,-0.29336968064308167,-0.08302041888237,-0.10018068552017212,0.24812133610248566,-0.1949155330657959,0.031817883253097534,0.2828197777271271,0.04229266196489334,-0.29019519686698914,-0.16746385395526886,0.028371961787343025,0.15864363312721252,0.1057683527469635,0.25676867365837097,-0.2500114142894745,0.1494807004928589,-0.01137425284832716,0.08863188326358795,0.23904253542423248,-0.14036008715629578,-0.037729471921920776,0.31733429431915283,0.002510104328393936,-0.2669451832771301,-0.29759401082992554,0.2460777461528778},{0.08897532522678375,0.18807274103164673,0.13336654007434845,0.1437089741230011,-0.13225314021110535,-0.363316148519516,0.1683606058359146,-0.23723529279232025,-0.053961075842380524,-0.2126442939043045,0.2985716462135315,-0.12558726966381073,-0.034290652722120285,-0.09751507639884949,0.29302161931991577,0.13850408792495728,0.08978739380836487,-0.02972538210451603,-0.3001730144023895,0.04008419066667557,-0.2823323607444763,-0.041825465857982635,-0.21645978093147278,0.18391869962215424,0.06193633750081062,-0.05575605481863022,0.19436945021152496,0.004364787135273218,0.09180205315351486,-0.19306552410125732,-0.2036564201116562,0.2779770791530609},{0.20907175540924072,0.41998744010925293,-0.14541427791118622,0.25887155532836914,-0.012134861201047897,0.1578039675951004,0.4888926148414612,-0.3054451644420624,-0.10091742128133774,-0.0915420800447464,0.19251906871795654,-0.14186589419841766,-0.016995126381516457,0.30947357416152954,-0.06823110580444336,0.2679746747016907,0.22926545143127441,0.23372630774974823,0.16610662639141083,0.15299633145332336,-0.2518579661846161,-0.25323569774627686,0.12660571932792664,0.2084069401025772,0.11347474902868271,0.29276344180107117,-0.37996622920036316,-0.35237714648246765,-0.05607794597744942,0.26034054160118103,0.041983574628829956,0.21211181581020355},{0.12040656059980392,0.044581089168787,0.058244261890649796,-0.30594372749328613,-0.11411300301551819,-0.22047221660614014,0.14819319546222687,0.3237557113170624,-0.2839932143688202,-0.2983124554157257,0.10915564000606537,-0.1352870613336563,0.034050822257995605,0.1808469444513321,-0.16972430050373077,0.3133230209350586,0.04190775752067566,-0.3132845461368561,0.1400282233953476,0.02367449179291725,0.2123536467552185,-0.36450690031051636,0.33969706296920776,-0.0407552607357502,0.1333540380001068,0.27289989590644836,0.2089536190032959,0.243155837059021,-0.10605959594249725,0.2956864833831787,0.006583840120583773,0.028884274885058403},{0.21732959151268005,0.04576703906059265,0.3620094358921051,-0.3453579246997833,0.16823188960552216,-0.15644551813602448,-0.1519133448600769,-0.28202569484710693,0.03515305742621422,0.3936949372291565,0.18244774639606476,-0.025049515068531036,0.12802937626838684,0.14072532951831818,0.0225134938955307,0.31047022342681885,-0.05632305145263672,-0.3002382516860962,-0.09462712705135345,0.005491968244314194,-0.16372142732143402,0.13843534886837006,0.029786324128508568,-0.199887216091156,-0.11721765249967575,-0.2013612538576126,-0.1571372151374817,0.043587472289800644,-0.20355254411697388,0.010346046648919582,0.32510170340538025,0.3796476125717163},{0.23234084248542786,0.07295765727758408,-0.21062436699867249,-0.34704846143722534,-0.2437635064125061,-0.2851027846336365,0.22633931040763855,0.09690237790346146,-0.1574094146490097,0.2013804316520691,-0.3720272183418274,-0.06159455329179764,-0.13799667358398438,-0.13963432610034943,-0.1402912735939026,-0.23027116060256958,-0.4046349823474884,-0.287984699010849,0.12926870584487915,0.16954857110977173,-0.07719139009714127,0.34018418192863464,-0.1869736611843109,-0.12246492505073547,0.19355899095535278,0.326649010181427,0.21024061739444733,-0.1966712772846222,-0.16449667513370514,-0.3101710081100464,-0.26420506834983826,0.09934118390083313},{-0.059650275856256485,-0.07956846803426743,-0.19337594509124756,0.022516166791319847,0.2238667756319046,-0.12750625610351562,0.19315031170845032,0.36507049202919006,0.1418135017156601,0.2440805286169052,-0.11786734312772751,-0.31471309065818787,0.0985713079571724,0.12623818218708038,0.20020370185375214,0.19194594025611877,-0.37839940190315247,-0.20652157068252563,-0.14258204400539398,0.19750802218914032,-0.06036335229873657,-0.297863245010376,-0.25482746958732605,-0.0928812325000763,0.19184523820877075,0.2909594774246216,0.25947925448417664,0.08845718204975128,-0.16197669506072998,-0.1453276127576828,0.07407404482364655,-0.1920524388551712},{0.2781599164009094,-0.0503925122320652,-0.294380247592926,-0.09006313979625702,0.021573545411229134,0.058630283921957016,-0.04518987983465195,-0.0793699249625206,0.15352562069892883,0.3547495901584625,-0.36351296305656433,0.27938589453697205,0.17481613159179688,-0.2727166712284088,0.049860306084156036,-0.270126074552536,-0.10024022310972214,0.18402661383152008,-0.14769792556762695,0.3921927511692047,0.12390852719545364,-0.14110492169857025,-0.2265377789735794,-0.13757042586803436,-0.3591102659702301,0.10001081973314285,0.007402581162750721,0.22981394827365875,-0.037314217537641525,0.132713183760643,0.34104210138320923,-0.20302435755729675},{0.16686761379241943,-0.1840008944272995,0.12828336656093597,-0.06812912970781326,-0.22262556850910187,-0.25454458594322205,0.19081012904644012,-0.2619900405406952,0.20805047452449799,0.1707935631275177,-0.16978564858436584,0.13782183825969696,-0.050504133105278015,0.0483119860291481,-0.13365329802036285,-0.06058862432837486,0.09289833158254623,-0.08591306954622269,-0.018895719200372696,-0.17479829490184784,0.03636658564209938,-0.19403330981731415,-0.18818143010139465,-0.3722377419471741,0.015099748969078064,-0.211455300450325,0.10943302512168884,0.1230492815375328,0.21818885207176208,0.08080561459064484,0.032496482133865356,0.005731706973165274},{0.2026916742324829,-0.11803862452507019,-0.27361974120140076,0.31399646401405334,-0.15875209867954254,0.09786869585514069,0.3566909730434418,-0.30215343832969666,-0.04279110208153725,-0.32712286710739136,-0.23119501769542694,-0.327613890171051,-0.27911376953125,-0.2632845640182495,0.04698459804058075,0.37424784898757935,0.29940125346183777,-0.19415384531021118,0.05471644923090935,-0.11320492625236511,-0.13518185913562775,-0.17851361632347107,-0.2445879727602005,0.2015395164489746,-0.15020997822284698,-0.0873790755867958,-0.15725676715373993,-0.12543129920959473,-0.12883499264717102,-0.11706149578094482,-0.15964899957180023,-0.2158840149641037},{0.24826659262180328,0.40748703479766846,-0.205634206533432,-0.12640392780303955,-0.020618299022316933,0.3578452467918396,0.2677963972091675,0.09508641064167023,-0.009757603518664837,-0.2621282637119293,0.04113611951470375,0.18949270248413086,-0.04419558495283127,0.0041190749034285545,0.12536141276359558,0.1177690401673317,-0.1485222429037094,-0.025045283138751984,-0.2289562225341797,-0.18547172844409943,-0.048470135778188705,0.08187080919742584,-0.0815325602889061,-0.31939268112182617,-0.16390781104564667,0.08182288706302643,0.09969042241573334,-0.16904528439044952,0.11314070969820023,-0.19140654802322388,0.23158471286296844,-0.1825529932975769},{-0.33746129274368286,-0.1274445801973343,0.17937129735946655,-0.04841161146759987,0.21157921850681305,0.13718195259571075,0.17991027235984802,-0.20651717483997345,-0.006099811289459467,0.10343128442764282,0.22704847157001495,0.12376869469881058,-0.13578851521015167,0.02405601367354393,-0.24088017642498016,-0.22834797203540802,0.20863164961338043,-0.0962064117193222,0.06457255035638809,0.10607929527759552,0.26591306924819946,0.23559395968914032,0.05725057050585747,-0.28415414690971375,0.07286836206912994,0.0403401143848896,0.3044857680797577,0.24502508342266083,-0.06565719842910767,-0.17798928916454315,0.08988972008228302,0.20960578322410583},{0.2056695967912674,-0.10681084543466568,-0.2025531381368637,0.0073982770554721355,-0.08359695971012115,0.2856924533843994,-0.2468757927417755,0.12589314579963684,-0.36492079496383667,-0.31320327520370483,0.16282598674297333,0.0011457661166787148,-0.08224266767501831,-0.1554223597049713,-0.05619083717465401,-0.09255761653184891,0.14986462891101837,0.03078104741871357,-0.07423805445432663,0.18425828218460083,0.07130197435617447,-0.07316146790981293,0.11765370517969131,0.25346678495407104,0.10509180277585983,-0.32211750745773315,-0.08269057422876358,0.09999243170022964,-0.1021973267197609,-0.05948740616440773,0.15769357979297638,0.15347959101200104},{0.2607179582118988,0.22887873649597168,0.1682385355234146,-0.08402600139379501,-0.2619710862636566,0.07314974069595337,-0.1311822086572647,0.21702702343463898,0.11088500916957855,0.03489400073885918,-0.10356899350881577,0.07350189983844757,0.17295579612255096,0.18577131628990173,-0.07098562270402908,-0.2884303033351898,-0.348308801651001,0.339162677526474,0.3941303491592407,-0.33093833923339844,-0.2349291443824768,0.2667628228664398,0.191279336810112,-0.21489404141902924,0.18245866894721985,0.004280728287994862,0.30994829535484314,0.07191555947065353,0.13554517924785614,-0.0027501306030899286,-0.21692733466625214,0.1827104389667511},{-0.39103710651397705,-0.01682574488222599,-0.23023991286754608,-0.04171082004904747,-0.11693159490823746,-0.24560019373893738,-0.007870904169976711,-0.04450051113963127,-0.12313303351402283,0.15553806722164154,0.22634880244731903,0.006373616401106119,0.04425937309861183,-0.3272337317466736,-0.2643392086029053,0.08119960129261017,-0.2725275456905365,0.02250361442565918,-0.20007960498332977,-0.011176927015185356,-0.04805413633584976,0.2171722948551178,0.21028690040111542,-0.019600477069616318,-0.13871605694293976,-0.04102085530757904,0.0869421660900116,0.18277351558208466,0.04847534000873566,-0.25453492999076843,-0.10005959868431091,-0.07274358719587326},{0.002441548276692629,0.32777127623558044,0.040941640734672546,0.16539940237998962,0.17531633377075195,0.09852717071771622,0.13774041831493378,0.13624919950962067,-0.3361000716686249,0.06761490553617477,-0.09779959917068481,0.18149609863758087,-0.2468462586402893,0.4074280858039856,-0.02899269387125969,-0.03325898200273514,0.1008242666721344,0.0997081995010376,-0.03926005959510803,0.256886750459671,-0.31879767775535583,0.0735970288515091,-0.06518951058387756,0.23512522876262665,0.11816170811653137,0.07890069484710693,0.17553679645061493,-0.13109835982322693,0.27067652344703674,0.25209659337997437,-0.09765715897083282,-0.16963127255439758},{-0.21110515296459198,0.3351918160915375,0.031360600143671036,0.022568056359887123,-0.24202309548854828,0.26932990550994873,0.01638011820614338,0.09649080783128738,0.049513787031173706,-0.3276272416114807,-0.10039401054382324,-0.2814215123653412,-0.12214413285255432,0.1458636373281479,0.042247921228408813,-0.18485714495182037,0.10572165995836258,0.10196609795093536,0.286792516708374,0.1312466412782669,-0.17351007461547852,-0.2970712184906006,-0.09268908947706223,0.36682114005088806,0.3877333104610443,0.21366214752197266,0.11985515058040619,-0.05127715691924095,0.03269996866583824,0.047690290957689285,-0.32630348205566406,0.23287886381149292},{-0.10167264938354492,-0.04304559901356697,-0.11861366778612137,-0.2479666918516159,0.08110465109348297,0.11755085736513138,0.169514462351799,0.013178023509681225,0.27207496762275696,0.08655930310487747,0.14894764125347137,-0.1675318330526352,-0.026146678254008293,-0.08317036181688309,-0.2980574071407318,-0.14105430245399475,-0.2983877658843994,-0.18225441873073578,-0.11539922654628754,-0.3128625154495239,-0.08176607638597488,0.13707169890403748,0.04285242035984993,0.22707585990428925,0.19492439925670624,0.040231045335531235,0.2961954176425934,-0.39025962352752686,-0.13931012153625488,-0.16515317559242249,0.016293173655867577,-0.029386140406131744}};
static const float action_parameterization_distribution_linear_weight[32][4] = {{0.3064926266670227,-0.06318370997905731,-0.33861714601516724,-0.21014311909675598},{0.09738293290138245,-0.178687185049057,-0.0711933970451355,0.38567522168159485},{0.047554418444633484,0.037188462913036346,-0.20943716168403625,-0.15772390365600586},{-0.32088392972946167,0.03649337962269783,0.04162629321217537,0.16361932456493378},{-0.3386348783969879,0.04117164388298988,-0.1394382268190384,-0.31694597005844116},{0.24407930672168732,-0.2632359564304352,-0.38854798674583435,0.15714076161384583},{0.11794298142194748,0.06345053017139435,-0.07572674751281738,0.4580812156200409},{0.4301013648509979,0.32077234983444214,0.14705361425876617,0.08367446064949036},{-0.3983168303966522,0.2947944700717926,0.19129230082035065,0.006382201798260212},{-0.2225896418094635,-0.24937555193901062,-0.211387038230896,-0.30165883898735046},{-0.005861359182745218,0.3142600357532501,-0.24705453217029572,-0.28012731671333313},{-0.19172999262809753,-0.3014203906059265,0.19645658135414124,0.15129442512989044},{0.269574910402298,0.2936099171638489,-0.1572140008211136,-0.43468257784843445},{0.36259669065475464,-0.057946935296058655,-0.15732313692569733,0.05951061472296715},{0.3212363123893738,0.34623727202415466,-0.46955665946006775,-0.32690051198005676},{0.012982742860913277,0.11837684363126755,-0.2713778018951416,0.043478984385728836},{-0.37951338291168213,-0.15990900993347168,-0.4435745179653168,-0.34073278307914734},{-0.1208590492606163,0.18527905642986298,0.11802958697080612,0.1687774807214737},{0.08189716935157776,0.32407885789871216,0.2345331758260727,0.08307144790887833},{-0.06161371245980263,-0.3655704855918884,0.1293543130159378,-0.2210179567337036},{0.11268563568592072,0.1527867466211319,-0.26485124230384827,-0.15976767241954803},{-0.18612894415855408,0.23857451975345612,0.42629390954971313,-0.22984442114830017},{0.3701649010181427,-0.024912457913160324,-0.26304343342781067,-0.12544924020767212},{0.29137763381004333,0.2206176221370697,-0.18575787544250488,-0.04959220066666603},{0.4031965136528015,0.3826107680797577,0.22199025750160217,0.11127389967441559},{0.3411667048931122,-0.29177480936050415,0.2455807626247406,-0.05858393758535385},{-0.19597023725509644,0.3208605945110321,0.3373846113681793,0.11042787879705429},{-0.17137958109378815,-0.26058048009872437,-0.05801196023821831,-0.34224817156791687},{-0.30386313796043396,0.21450911462306976,-0.2950471043586731,0.36467623710632324},{0.14862531423568726,-0.3111981153488159,-0.456712931394577,-0.16416707634925842},{-0.0379595123231411,-0.15324664115905762,0.0010407334193587303,0.1196926087141037},{0.13931314647197723,0.0569397397339344,-0.3136970102787018,-0.24140076339244843}};
static const float actor_encoder_self_encoder_0_bias[16] = {0.02014175057411194,-0.0912427008152008,-0.00019805158080998808,-0.04009043797850609,0.09516645222902298,0.033751171082258224,0.05043472349643707,0.04089714586734772,0.1050531417131424,0.04228796809911728,0.06972501426935196,0.002696329029276967,-0.05214540660381317,0.011139879934489727,-0.1307954043149948,-0.01690756343305111};
static const float actor_encoder_self_encoder_2_bias[16] = {0.027741460129618645,0.0009754523052833974,0.018951481208205223,-0.020351262763142586,0.0149167999625206,0.021036555990576744,-0.016592377796769142,0.034718818962574005,-0.020113976672291756,-0.025399833917617798,0.012119152583181858,0.025090310722589493,-0.03778117150068283,-0.0006420626887120306,-0.0218561589717865,-0.003987723495811224};
static const float actor_encoder_feed_forward_0_bias[32] = {-0.012685044668614864,0.021165234968066216,0.03123827837407589,-0.012980488128960133,-0.039268724620342255,0.015980185940861702,0.01895194500684738,0.01575024612247944,-0.0009027590276673436,-0.035641491413116455,-0.028575493022799492,0.0009760892717167735,0.0007132919272407889,-0.024074746295809746,0.022258995100855827,0.015560688450932503,-0.04273601248860359,-0.007278818637132645,0.008379177190363407,-0.03396393358707428,-0.01937318779528141,-0.0030641399789601564,-0.02344919927418232,0.022004010155797005,0.027141442522406578,-0.0255587175488472,-0.0035834764130413532,-0.037051234394311905,0.0002556144609116018,-0.001210399903357029,0.03821701556444168,-0.028178907930850983};
static const float action_parameterization_distribution_linear_bias[4] = {0.020843524485826492,0.019215738400816917,0.01260282937437296,0.025778476148843765};
static const float actor_encoder_neighbor_encoder_embedding_mlp_0_weight[3][8] = {{-1.0530186891555786,1.0613667964935303,-0.18755817413330078,0.6282792687416077,0.7436965703964233,-0.014100934378802776,0.6207965612411499,-0.8221315741539001},{0.3382953703403473,0.6490414142608643,1.1873987913131714,-0.590337872505188,0.26540687680244446,-0.3867320418357849,0.19587565958499908,0.6863990426063538},{0.3284076154232025,0.5169113874435425,-0.5596040487289429,-0.050812095403671265,-0.3047058880329132,0.3313647508621216,-1.2519636154174805,0.025929559022188187}};
static const float actor_encoder_neighbor_encoder_embedding_mlp_2_weight[8][8] = {{0.34685736894607544,-0.7367794513702393,0.17178480327129364,-0.6856496930122375,1.0763856172561646,-0.6252858638763428,0.03616678714752197,0.39844101667404175},{0.2598486840724945,-0.21605901420116425,-0.36389997601509094,-0.6308271884918213,-0.342544287443161,0.7025375962257385,-0.7141960859298706,-0.4757542312145233},{-0.2781651020050049,-0.024708641692996025,-0.34400057792663574,-0.5556723475456238,1.0004384517669678,0.8437077403068542,0.5860052704811096,0.06379204243421555},{-0.25078657269477844,0.391724169254303,-0.5492984056472778,-0.13269191980361938,-1.139338493347168,0.08337162435054779,-0.2509596347808838,0.6568831205368042},{-0.7133592367172241,-0.025283601135015488,0.23070909082889557,0.12972214818000793,-0.5879473686218262,0.5136315226554871,-0.05538948252797127,0.5381146669387817},{0.3773505389690399,-0.42434483766555786,-0.34167370200157166,-0.2361256182193756,-0.04543541744351387,0.349409282207489,-0.47029030323028564,-0.11043797433376312},{0.32186582684516907,-0.3886822462081909,0.07825945317745209,0.16086098551750183,-0.3584004342556,0.7957894802093506,-0.12397937476634979,0.741614818572998},{-0.17088830471038818,0.41960564255714417,-0.08375424891710281,-0.23094095289707184,1.0541560649871826,-0.6080890893936157,0.20163962244987488,0.13375715911388397}};
static const float actor_encoder_neighbor_encoder_embedding_mlp_0_bias[8] = {0.007688176352530718,-0.007296745665371418,-0.0010914461454376578,-0.032901059836149216,0.011421040631830692,0.005403751041740179,-0.006299640983343124,0.015024040825664997};
static const float actor_encoder_neighbor_encoder_embedding_mlp_2_bias[8] = {-0.0023012065794318914,-0.015879452228546143,-0.019930895417928696,-0.013974732719361782,-0.005745003931224346,-0.0075240228325128555,0.0012450122740119696,-0.0068059018813073635};

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

    // Concat self_embed and neighbor_embed
    for (int i = 0; i < D_SELF; i++) {
        output_embeds[i] = output_1[i];
    }
    for (int i = 0; i < D_NBR; i++) {
        output_embeds[D_SELF + i] = neighbor_embeds[i];
    }

    // Feedforward layer
    for (int i = 0; i < self_structure[2][1]; i++) {
        output_2[i] = 0;
        for (int j = 0; j < self_structure[2][0] ; j++) {
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

void neighborEmbedder(const float neighbor_inputs[NEIGHBORS*NBR_OBS_DIM]) {
    for (int n = 0; n < NEIGHBORS; n++) {            
        for (int i = 0; i < nbr_structure[0][1]; i++) {
            nbr_output_0[n][i] = 0; 
            for (int j = 0; j < nbr_structure[0][0]; j++) {
                nbr_output_0[n][i] += neighbor_inputs[n*NBR_OBS_DIM + j] * actor_encoder_neighbor_encoder_embedding_mlp_0_weight[j][i]; 
            }
            nbr_output_0[n][i] += actor_encoder_neighbor_encoder_embedding_mlp_0_bias[i];
            nbr_output_0[n][i] = tanhf(nbr_output_0[n][i]);
        }
    }

    for (int n = 0; n < NEIGHBORS; n++) {
        for (int i = 0; i < nbr_structure[1][1]; i++) {
            nbr_output_1[n][i] = 0;
            for (int j = 0; j < nbr_structure[1][0]; j++) {
                nbr_output_1[n][i] += nbr_output_0[n][j] * actor_encoder_neighbor_encoder_embedding_mlp_2_weight[j][i];
            }
            nbr_output_1[n][i] += actor_encoder_neighbor_encoder_embedding_mlp_2_bias[i];
            nbr_output_1[n][i] = tanhf(nbr_output_1[n][i]);
        }
    }

    // Average over number of neighbors
    for (int i = 0; i < D_NBR; i++) {
        neighbor_embeds[i] = 0;
        for (int n = 0; n < NEIGHBORS; n++) {
            neighbor_embeds[i] += nbr_output_1[n][i];
        }
        neighbor_embeds[i] /= NEIGHBORS;
    }
}