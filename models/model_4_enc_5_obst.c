
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
static const float actor_encoder_self_encoder_0_weight[18][16] = {{0.23587031662464142,-0.32420462369918823,0.1356368362903595,0.21517035365104675,-0.09065943211317062,-0.17326919734477997,0.00615980289876461,0.17960615456104279,-0.30351486802101135,-0.26576393842697144,-0.29048338532447815,-0.02892332710325718,0.13978494703769684,-0.2226254791021347,0.012481417506933212,0.29405903816223145},{-0.054609645158052444,0.2912192940711975,-0.15682779252529144,0.4282090961933136,0.12897780537605286,-0.3162083029747009,-0.1244027391076088,-0.23240257799625397,0.3583656847476959,-0.3780338764190674,0.22199656069278717,-0.33188074827194214,0.07904255390167236,-0.03255399689078331,-0.10576015710830688,0.2195788323879242},{-0.1425560563802719,-0.11614344269037247,0.6956368684768677,0.20569239556789398,0.1708521693944931,0.05136978626251221,0.17865878343582153,0.21176309883594513,-0.2314014434814453,-0.6345805525779724,-0.34451478719711304,0.3052360415458679,0.16956879198551178,-0.5014026761054993,0.05662851780653,-0.6519646048545837},{0.11835716664791107,-0.392245888710022,0.12568151950836182,0.08215171843767166,-0.18125376105308533,0.18325138092041016,0.06300704926252365,-0.12743937969207764,-0.14608018100261688,-0.11652754992246628,-0.03040890395641327,-0.009544149041175842,0.41858547925949097,0.225029855966568,0.08471686393022537,0.17304383218288422},{-0.004352949094027281,0.05328778922557831,-0.03354723006486893,0.15923750400543213,0.2782844007015228,-0.19725558161735535,0.1250077188014984,-0.05085443705320358,0.2060350626707077,-0.0667041763663292,0.20733094215393066,0.42109256982803345,0.30347633361816406,0.22323863208293915,0.19394031167030334,-0.051226336508989334},{0.14866074919700623,-0.18187275528907776,-0.15124037861824036,0.20924638211727142,0.2636066973209381,0.2772975564002991,-0.3523336946964264,0.13165922462940216,0.2968573272228241,-0.24682265520095825,-0.18870630860328674,0.07078496366739273,0.05849241092801094,-0.2603643238544464,0.019097566604614258,-0.4319978952407837},{-0.7091374397277832,0.004277142696082592,0.12550662457942963,0.21632954478263855,-0.10814429074525833,-0.2713182270526886,0.29327112436294556,-0.004817810840904713,-0.2614883780479431,0.02290668524801731,0.021833445876836777,0.3542940020561218,-0.7984947562217712,0.2510627210140228,-0.2845803499221802,-0.2531954348087311},{0.2920861542224884,0.13240785896778107,0.22894367575645447,-0.02048412524163723,-0.2077179104089737,-0.004658603109419346,0.25866764783859253,-0.369884729385376,0.29056689143180847,0.15624262392520905,-0.021405218169093132,0.10826341807842255,0.10032253712415695,0.3141081631183624,-0.5969074368476868,0.3645298480987549},{0.25402069091796875,-0.6316771507263184,0.18797697126865387,-0.10501942038536072,-0.055942337960004807,0.5099911093711853,0.20890165865421295,-0.3875085413455963,0.024721166118979454,-0.14796032011508942,-0.5032079815864563,0.32776620984077454,0.027240071445703506,0.08159173280000687,0.1359545737504959,0.6409111022949219},{-0.4492054283618927,-0.5539819598197937,-0.43752521276474,-0.3352257013320923,0.4496075510978699,-0.5230636596679688,-0.3487975597381592,0.13020017743110657,0.1309819519519806,-0.43968573212623596,0.3791864216327667,0.1036662757396698,-0.01438929419964552,-0.18523187935352325,0.4923556447029114,-0.4427931010723114},{-0.5177931189537048,-0.0662677213549614,-0.2827012836933136,0.4560856819152832,-0.3551139235496521,-0.158035546541214,0.5449896454811096,-0.45003992319107056,-0.3118513822555542,0.0393807552754879,0.005440607201308012,0.4001505672931671,-0.7661513090133667,-0.22955188155174255,-0.196610227227211,0.11545532941818237},{-0.7886874079704285,-0.053794439882040024,0.06590813398361206,0.3185542821884155,0.7922048568725586,-0.6504114866256714,0.5300566554069519,-0.03808030113577843,0.18112929165363312,0.2574892044067383,0.3387904167175293,0.37762874364852905,0.0870596319437027,0.6070759892463684,-0.2552708685398102,0.20436525344848633},{0.45135781168937683,0.4357910454273224,-0.019821446388959885,-0.21137911081314087,0.10510874539613724,0.09816117584705353,0.29529935121536255,-0.023981090635061264,0.18335290253162384,0.38760650157928467,0.3480295240879059,0.07390826940536499,-0.3014170527458191,-0.5577414631843567,0.4195297360420227,-0.1600460559129715},{0.2030504196882248,-0.4769858717918396,0.2668958902359009,-0.38556230068206787,-0.33554160594940186,0.03173283487558365,-0.5195557475090027,-0.16744722425937653,-0.31087496876716614,-0.6346843242645264,-0.07782189548015594,-0.2858637273311615,-0.41777899861335754,-0.12204591184854507,0.31230080127716064,0.169877290725708},{-0.6170132756233215,-0.21633152663707733,-0.002190352650359273,-0.1842462122440338,-0.24368463456630707,-0.30907315015792847,0.4769397974014282,0.09171108901500702,-0.22860780358314514,-0.0904427245259285,0.06408858299255371,-0.03371420130133629,-0.33643531799316406,-0.017918193712830544,0.267985075712204,0.29908138513565063},{0.24173763394355774,-0.09751582890748978,0.024990659207105637,0.11048524081707001,-0.19743841886520386,0.20753571391105652,-0.3434462249279022,-0.2654734253883362,-0.2760356664657593,-0.05630497634410858,-0.23748032748699188,-0.35297346115112305,-0.12935984134674072,-0.10949823260307312,0.15440000593662262,-0.017877863720059395},{-0.054384082555770874,-0.2651544511318207,0.1502288430929184,0.17682665586471558,0.06209802255034447,-0.06865765899419785,-0.1989615112543106,-0.31043562293052673,-0.27939966320991516,-0.031932804733514786,0.018622076138854027,0.2943302094936371,0.14438076317310333,0.042171966284513474,-0.01637929491698742,0.1619209349155426},{-0.16558438539505005,-0.10225316137075424,0.5122884511947632,-0.1928267925977707,0.06781084090471268,-0.0869380459189415,0.037298865616321564,0.2702454924583435,-0.4414689540863037,-0.17021451890468597,0.16900686919689178,0.2640412747859955,-0.24714790284633636,-0.2657351791858673,0.18670615553855896,-0.08066287636756897}};
static const float actor_encoder_self_encoder_2_weight[16][16] = {{-0.14597369730472565,0.044801875948905945,0.19554217159748077,-0.4847794473171234,0.015405353158712387,0.11499401181936264,-0.08190836757421494,0.12143245339393616,0.21100202202796936,0.37859463691711426,-0.23120038211345673,-0.1638306975364685,0.24473096430301666,-0.10966434329748154,0.24537888169288635,-0.15497927367687225},{0.04723335802555084,-0.4693385064601898,0.22426316142082214,-0.1975233256816864,-0.2796255350112915,-0.018708115443587303,0.24007461965084076,-0.0070265368558466434,0.3457071781158447,-0.13765424489974976,-0.20458339154720306,-0.21233554184436798,-0.03132883459329605,0.26740214228630066,0.1291402280330658,-0.10934635996818542},{-0.20751607418060303,-0.14288176596164703,-0.12807300686836243,-0.2594296932220459,-0.2611338198184967,-0.02465260960161686,0.19736969470977783,0.25716614723205566,-0.03951901197433472,-0.1518280804157257,-0.04347001761198044,0.22505544126033783,0.23782706260681152,-0.34795504808425903,0.38456588983535767,-0.24080044031143188},{-0.01724807545542717,0.1419777274131775,-0.30750662088394165,-0.06005622446537018,-0.47414630651474,0.07912826538085938,0.05788054317235947,-0.2644721269607544,-0.05447385087609291,-0.39712151885032654,0.3058030903339386,0.06838534772396088,0.04769844934344292,-0.1519644856452942,0.12927861511707306,0.3810986578464508},{-0.02579011209309101,-0.13503149151802063,0.0365142785012722,0.3950810134410858,-0.4074392318725586,-0.02379044145345688,-0.005976078566163778,-0.12041031569242477,-0.23764513432979584,-0.18729691207408905,0.36966606974601746,-0.26023298501968384,-0.06450062990188599,0.024996723979711533,-0.3482047915458679,-0.3695314824581146},{-0.34185516834259033,0.3437441289424896,0.05458875745534897,-0.15534433722496033,0.41481882333755493,-0.3840009868144989,-0.14999271929264069,-0.15989309549331665,-0.0007732557132840157,0.012085257098078728,0.2521270513534546,-0.22925157845020294,0.26379936933517456,0.1738949865102768,0.3246610462665558,0.12046634405851364},{0.13228091597557068,-0.06160757318139076,0.30750375986099243,0.3022722005844116,0.16194100677967072,0.5554496049880981,0.14735203981399536,0.31434324383735657,-0.07731504738330841,-0.04397508502006531,0.29658302664756775,0.0773521363735199,-0.4622333347797394,-0.13202612102031708,0.14685362577438354,0.3532170057296753},{0.17132727801799774,-0.25956013798713684,0.2455248087644577,-0.16348595917224884,-0.05012698099017143,-0.1261506974697113,-0.16439078748226166,-0.12323083728551865,-0.031005242839455605,-0.1762789487838745,0.16341532766819,-0.3346002697944641,-0.17357850074768066,-0.06636153906583786,0.2616092562675476,-0.40550732612609863},{0.044536978006362915,0.1273850053548813,0.27612632513046265,-0.11422983556985855,-0.2964315712451935,0.10989359021186829,0.27167651057243347,0.06546877324581146,0.11760365217924118,0.3379179835319519,0.06410831212997437,0.03954356908798218,-0.01343550905585289,0.4101165533065796,-0.029924649745225906,0.20179924368858337},{0.025235764682292938,0.11830031126737595,-0.29793816804885864,0.056413646787405014,-0.04970061406493187,-0.07950963824987411,-0.3310259282588959,0.39085710048675537,0.25017470121383667,-0.18494533002376556,-0.21922588348388672,-0.056437768042087555,-0.2901262044906616,0.09547999501228333,0.1770218014717102,-0.2055141180753708},{0.39051464200019836,-0.031010087579488754,-0.42269545793533325,-0.17071227729320526,-0.404346227645874,0.14498193562030792,0.21762894093990326,0.17199048399925232,-0.09362468123435974,0.1626751869916916,-0.1503775268793106,0.08471845090389252,-0.3157884478569031,0.31337839365005493,0.19703452289104462,0.0013132524909451604},{-0.3219172954559326,0.3155052065849304,-0.32680222392082214,-0.28885966539382935,0.18369215726852417,0.27396172285079956,0.2678733468055725,-0.3251326084136963,0.13847258687019348,0.42702850699424744,0.3332231938838959,0.02385489083826542,-0.28408440947532654,0.20022903382778168,0.11605945974588394,0.31720954179763794},{-0.2098437398672104,-0.35339951515197754,0.1640491783618927,0.19750717282295227,-0.14796073734760284,-0.34620043635368347,0.29113101959228516,-0.2942902147769928,0.1638987809419632,-0.014260618947446346,-0.2050866186618805,-0.48947378993034363,0.057579346001148224,-0.49379223585128784,-0.21751384437084198,-0.2679702639579773},{-0.18521322309970856,0.2203930914402008,-0.43541255593299866,-0.2032555788755417,-0.12976539134979248,0.22283297777175903,0.09208205342292786,-0.21892911195755005,-0.09395889192819595,-0.1327003836631775,-0.38778799772262573,0.21369339525699615,-0.06152528524398804,0.08495007455348969,-0.08063894510269165,0.4238446056842804},{0.3939894139766693,0.34562060236930847,-0.28208571672439575,-0.5407158136367798,-0.16405603289604187,-0.48617756366729736,0.38305869698524475,0.15604472160339355,-0.038904789835214615,0.05504140257835388,0.07246627658605576,0.33477485179901123,-0.3621904253959656,0.11873836070299149,-0.22263070940971375,-0.3015007972717285},{-0.47738030552864075,0.3904879689216614,-0.39553698897361755,0.41656386852264404,0.2794894874095917,-0.04882775619626045,-0.4283472001552582,-0.4276716709136963,-0.1811634600162506,-0.0345541313290596,-0.45977091789245605,0.12159997224807739,0.1615925282239914,0.14278121292591095,-0.03587017208337784,-0.05741988122463226}};
static const float actor_encoder_feed_forward_0_weight[20][20] = {{-0.3755606412887573,-0.20668154954910278,-0.20012690126895905,0.30491602420806885,0.28745195269584656,-0.21782512962818146,-0.12707120180130005,-0.3741290271282196,0.089126817882061,0.2827374339103699,-0.07005064189434052,-0.18618661165237427,-0.2768228054046631,0.16224664449691772,-0.4203450381755829,0.2667304277420044,-0.0567922480404377,0.13214251399040222,0.05640391260385513,0.17098468542099},{0.014222958125174046,0.2805730104446411,0.2989795207977295,0.29358193278312683,0.10473506897687912,-0.23456762731075287,0.20545409619808197,-0.18791289627552032,0.2105008214712143,-0.1395549327135086,0.02662596106529236,0.1391593962907791,-0.1777857095003128,-0.360548198223114,0.06587634980678558,-0.16983121633529663,0.38365116715431213,0.0867343470454216,-0.21280021965503693,0.4212128221988678},{0.015687042847275734,-0.24560634791851044,-0.0045429798774421215,-0.14515863358974457,0.03940078243613243,0.24069347977638245,-0.16012008488178253,0.4432831406593323,-0.2665114998817444,-0.019647512584924698,-0.4383050799369812,0.11960360407829285,0.21701066195964813,-0.2747454345226288,-0.17123714089393616,-0.01120383758097887,-0.34168189764022827,0.27748820185661316,-0.13700659573078156,-0.10870730131864548},{0.3738240897655487,0.21446560323238373,-0.06490734219551086,0.18983863294124603,-0.28178900480270386,0.0903598815202713,0.4947900176048279,-0.478318452835083,-0.2983400523662567,-0.048634886741638184,0.05882273614406586,0.413141667842865,-0.0990048348903656,-0.19150151312351227,0.24724240601062775,0.4300263524055481,-0.20124977827072144,-0.28338924050331116,0.028513431549072266,0.1870550960302353},{-0.03349893167614937,-0.17157140374183655,-0.27719759941101074,0.0838526114821434,0.34188804030418396,0.3428824841976166,0.10668423771858215,-0.47150635719299316,0.22123651206493378,-0.0654401108622551,0.25098851323127747,0.14813578128814697,0.35166776180267334,-0.036326371133327484,-0.2538183629512787,0.27696940302848816,-0.11610723286867142,0.062052398920059204,-0.17310106754302979,0.3166297674179077},{0.36597710847854614,0.23485980927944183,0.027303066104650497,0.11689649522304535,-0.16474315524101257,-0.29882416129112244,-0.33516839146614075,0.06617770344018936,-0.29188525676727295,0.19077496230602264,0.2721400558948517,0.08047281205654144,0.20968309044837952,-0.2179156094789505,0.345026433467865,-0.0986417680978775,0.20341403782367706,-0.09183657914400101,0.2592204511165619,-0.24729618430137634},{-0.062203772366046906,-0.055556271225214005,0.12005247920751572,0.13374152779579163,0.30008819699287415,-0.5060998797416687,-0.011663742363452911,0.16778075695037842,-0.18984684348106384,-0.326335608959198,0.16196565330028534,-0.054207250475883484,-0.0766998752951622,-0.08091805875301361,0.36708059906959534,0.3138320744037628,0.10705466568470001,0.14254145324230194,-0.2053174525499344,-0.21181993186473846},{-0.037622738629579544,0.0872843861579895,-0.14090460538864136,0.3263867497444153,0.19507169723510742,-0.027927560731768608,0.08694011718034744,-0.30815133452415466,-0.04108959436416626,0.20795726776123047,-0.21553559601306915,0.21949391067028046,-0.1257927417755127,-0.22432976961135864,-0.09578517824411392,0.012456736527383327,0.28403908014297485,-0.18989135324954987,-0.22995737195014954,-0.31622540950775146},{0.17321112751960754,-0.12744514644145966,0.1337435394525528,0.37797999382019043,0.07842708379030228,0.1451943814754486,-0.3832034170627594,-0.1599997878074646,-0.26860666275024414,0.19545675814151764,-0.12292855978012085,-0.13919095695018768,-0.4472215175628662,-0.3376576900482178,0.2015792578458786,0.3086196184158325,-0.45345157384872437,0.1735091507434845,-0.22660209238529205,0.03821124881505966},{-0.38861194252967834,-0.2513154447078705,0.23287205398082733,-0.30190205574035645,-0.2934577763080597,0.08761362731456757,0.05252429470419884,0.08256515115499496,0.2763763964176178,0.1285204142332077,-0.18026849627494812,0.2960686981678009,0.16231723129749298,-0.48853063583374023,0.4029410779476166,-0.025983721017837524,-0.39163410663604736,0.21689356863498688,0.01776016317307949,0.28532475233078003},{0.3432084321975708,0.021383168175816536,0.17489822208881378,-0.28396642208099365,0.08355802297592163,-0.5507684350013733,0.3783976137638092,-0.08351865410804749,-0.4476240873336792,0.2760346531867981,-0.14309802651405334,-0.29085472226142883,0.3787885308265686,-0.42429450154304504,0.14050787687301636,-0.36809873580932617,-0.13309672474861145,0.17731647193431854,0.08241983503103256,-0.1273452192544937},{0.415038138628006,-0.266988605260849,0.11807592213153839,0.18912892043590546,-0.19693264365196228,-0.052895162254571915,0.16942530870437622,0.16972391307353973,0.1424652636051178,-0.13230809569358826,0.09779174625873566,-0.025113027542829514,0.10550417006015778,-0.1860899180173874,-0.4520091414451599,-0.08026620000600815,0.3375783860683441,-0.11504074186086655,-0.08503354340791702,0.1956881880760193},{-0.0860278531908989,-0.42036905884742737,0.152205228805542,-0.26290005445480347,0.06429574638605118,0.04040156677365303,-0.1077355220913887,0.19434641301631927,0.02583943121135235,-0.05906454846262932,0.22359512746334076,0.21939072012901306,-0.4590688645839691,0.011688091792166233,-0.1941988170146942,0.06572733074426651,-0.23796997964382172,0.06499633193016052,0.24289938807487488,0.2255953997373581},{0.09857264161109924,0.217383474111557,-0.16538646817207336,0.28026631474494934,0.010058538988232613,0.3778151273727417,0.02437696047127247,-0.3665030896663666,0.19044065475463867,0.08138176053762436,0.14951084554195404,0.23409757018089294,-0.1765582412481308,-0.09347634017467499,-0.1837768703699112,0.37111547589302063,0.12654638290405273,0.05498311668634415,-0.00905038695782423,-0.029424134641885757},{0.4435863792896271,-0.2349357306957245,0.05519542470574379,-0.12925110757350922,0.3138553202152252,0.025020543485879898,0.02871229499578476,-0.3603530526161194,-0.3021855652332306,-0.15463946759700775,0.20239372551441193,-0.1275581270456314,-0.11617126315832138,-0.086105115711689,0.5223701000213623,-0.38511499762535095,-0.22651879489421844,0.06031101197004318,0.05872297286987305,0.16036008298397064},{0.28546643257141113,-0.1745796799659729,0.3263419270515442,0.025887588039040565,-0.2525116503238678,0.2895328104496002,-0.22697323560714722,0.23056919872760773,-0.15965522825717926,-0.04980795457959175,-0.358833372592926,-0.18515320122241974,0.13490676879882812,-0.18878531455993652,-0.13697560131549835,0.36105021834373474,0.2367130070924759,-0.28692159056663513,0.1779402494430542,0.07991959899663925},{0.2412015199661255,-0.09553851187229156,-0.23823969066143036,0.2543216347694397,0.022991031408309937,-0.28542253375053406,0.41326969861984253,-0.3165309429168701,0.1168806403875351,0.027985122054815292,0.18080241978168488,-0.24592800438404083,-0.11822866648435593,0.33971044421195984,-0.05103645846247673,-0.177395299077034,-0.11943630874156952,-0.2929694652557373,0.38738274574279785,-0.34288060665130615},{0.057825665920972824,0.03859071433544159,0.04631826654076576,0.11034572869539261,0.016232827678322792,-0.12526987493038177,0.27413612604141235,-0.006660685408860445,-0.1854645162820816,-0.1716947853565216,0.2972242832183838,0.17689387500286102,-0.264577716588974,-0.1673755943775177,-0.13907647132873535,0.11948687583208084,-0.14027564227581024,-0.3102961480617523,0.3052084743976593,-0.372099906206131},{-0.23701569437980652,-0.13095200061798096,-0.11553606390953064,0.23727653920650482,-0.0431521013379097,-0.2573498785495758,0.12470542639493942,0.31437548995018005,0.1496172994375229,-0.4481564462184906,0.09876663982868195,0.2536836564540863,-0.2821972370147705,0.373417466878891,-0.12448202818632126,0.24961325526237488,-0.37262436747550964,-0.2582317292690277,0.038014061748981476,-0.4468879699707031},{-0.05045485123991966,-0.09255120903253555,-0.32445934414863586,0.05443795770406723,0.3470359444618225,0.3306640386581421,-0.15737707912921906,0.011140447109937668,0.14435027539730072,-0.3013697862625122,-0.2815314829349518,0.2694028317928314,0.3009941279888153,-0.27090179920196533,0.08794166147708893,0.03832918033003807,-0.3953501582145691,-0.05596986040472984,-0.20412643253803253,0.08614474534988403}};
static const float action_parameterization_distribution_linear_weight[20][4] = {{-0.47913628816604614,0.4851222634315491,-0.4246632754802704,0.10538722574710846},{0.19283857941627502,0.07727067917585373,0.360059529542923,0.37031444907188416},{0.06731119006872177,-0.2790418863296509,-0.0677388608455658,0.3378277122974396},{0.09978679567575455,0.10378026217222214,0.35748475790023804,-0.06691253930330276},{-0.1986708790063858,-0.07393546402454376,0.1492442786693573,-0.27599987387657166},{0.19859789311885834,0.3359161615371704,0.1545034945011139,0.47452694177627563},{0.3863552510738373,-0.5003703236579895,0.10269241034984589,-0.032924894243478775},{0.04601297527551651,0.11303724348545074,-0.425747811794281,-0.2551354467868805},{0.2263309508562088,0.08941157907247543,0.33035632967948914,0.2611939013004303},{-0.3766000270843506,0.052929285913705826,0.3373928666114807,0.1809655874967575},{0.14885155856609344,0.22577226161956787,-0.060895323753356934,0.1937030553817749},{0.3133115768432617,0.13812142610549927,0.16984674334526062,0.05054616555571556},{0.08891651034355164,-0.5006182789802551,-0.015705402940511703,-0.020396258682012558},{0.17626546323299408,-0.2692311704158783,0.27185794711112976,0.5392575860023499},{-0.1938745081424713,-0.18499477207660675,0.20608513057231903,0.44410455226898193},{-0.04668889194726944,-0.03219708800315857,0.23620256781578064,0.1502109169960022},{0.5107061862945557,0.157566636800766,-0.45885226130485535,0.37319403886795044},{0.2562096416950226,-0.05736421048641205,0.30869248509407043,-0.269091933965683},{-0.18351826071739197,-0.12759852409362793,0.14952823519706726,0.0664922446012497},{0.49099212884902954,0.322221577167511,0.29586848616600037,-0.3976835608482361}};
static const float actor_encoder_self_encoder_0_bias[16] = {-0.1466008573770523,-0.0451192669570446,-0.0621093325316906,0.026005219668149948,-0.26248088479042053,0.07006323337554932,0.15845759212970734,-0.02769169583916664,0.05134094879031181,-0.0038068052381277084,0.0558539554476738,0.13684824109077454,-0.32249772548675537,-0.022354792803525925,0.07500957697629929,-0.03015531599521637};
static const float actor_encoder_self_encoder_2_bias[16] = {0.18013378977775574,0.08007097989320755,-0.14721840620040894,0.06588663160800934,0.003586672944948077,9.162830247078091e-05,-0.017432374879717827,0.028380760923027992,0.020999139174818993,-0.06614278256893158,-0.03931047394871712,0.028804711997509003,0.016101203858852386,0.055178530514240265,0.16162338852882385,0.0804782435297966};
static const float actor_encoder_feed_forward_0_bias[20] = {0.0702531486749649,0.012853617779910564,-0.03719504922628403,0.05104575678706169,0.06351272761821747,-0.011515943333506584,0.06272511184215546,-0.180472731590271,-0.029557548463344574,0.08798659592866898,-0.024689393118023872,0.03252159059047699,0.011292414739727974,-0.0018850582418963313,-0.09720221906900406,0.1271122395992279,0.05718105286359787,0.025796040892601013,-0.015545792877674103,0.08509456366300583};
static const float action_parameterization_distribution_linear_bias[4] = {-0.006578474771231413,-0.0011572266230359674,0.04532087594270706,-0.024517826735973358};
static const float actor_encoder_obstacle_encoder_0_weight[16][4] = {{-0.2636898458003998,0.21369177103042603,-0.08269375562667847,0.044189102947711945},{-0.20541511476039886,-0.006837249267846346,0.26225677132606506,-0.16183501482009888},{0.5180198550224304,0.08597512543201447,-0.19555459916591644,-0.19412429630756378},{0.3433944880962372,-0.25692376494407654,0.5217351913452148,0.44130486249923706},{0.026498453691601753,-0.38220712542533875,-0.1811782270669937,-0.09407229721546173},{0.05759381130337715,0.33067837357521057,0.18850846588611603,0.031729403883218765},{-0.43390509486198425,0.09478490054607391,-0.43754786252975464,0.2649182379245758},{0.4320290982723236,-0.2728959321975708,0.4153822362422943,0.24720589816570282},{0.009384442120790482,0.05213551223278046,0.6805038452148438,0.0984029769897461},{-0.09744733572006226,-0.08243325352668762,0.6505943536758423,-0.36882007122039795},{-0.3397761285305023,-0.0027482251171022654,0.19815851747989655,-0.6687127351760864},{0.6289069056510925,0.4259154498577118,0.2886123061180115,-0.7757025361061096},{0.7288170456886292,-0.1978306919336319,0.2540746033191681,0.05600067228078842},{0.8239104151725769,0.6204798817634583,0.46110180020332336,-0.06597292423248291},{0.4692120850086212,0.39729148149490356,0.44981855154037476,-0.3291650116443634},{-0.07983426004648209,-0.07173604518175125,-0.3743155598640442,0.28544721007347107}};
static const float actor_encoder_obstacle_encoder_2_weight[4][4] = {{0.7687429189682007,-0.5924006700515747,-0.4180441200733185,-0.06100492551922798},{0.530451238155365,0.3650270104408264,-0.4115125834941864,-0.3370681703090668},{-0.15352413058280945,-0.7270212769508362,-0.7506844401359558,0.635759711265564},{0.08313319087028503,0.46056804060935974,-0.2016880214214325,0.5191721320152283}};
static const float actor_encoder_obstacle_encoder_0_bias[4] = {0.2096647173166275,0.04053357616066933,0.09256570041179657,-0.15805034339427948};
static const float actor_encoder_obstacle_encoder_2_bias[4] = {-0.04048643633723259,-0.16506710648536682,-0.11854938417673111,0.0014772320864722133};
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
