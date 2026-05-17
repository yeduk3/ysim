#include "MeshInspectorWindow.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include <cmath>
#include <cstdio>
#include <unordered_map>

namespace mesh_inspector {

static const ImVec4 kG100(0.063f,0.078f,0.102f,1), kG90(0.098f,0.122f,0.157f,1),
    kG60(0.420f,0.463f,0.518f,1), kG50(0.545f,0.584f,0.631f,1),
    kG40(0.690f,0.722f,0.757f,1), kG20(0.886f,0.906f,0.922f,1),
    kG10(0.933f,0.941f,0.949f,1), kG5(0.976f,0.980f,0.984f,1), kW(1,1,1,1);
static const float kP=24;

static int clothBI(bool g){return g?2:1;}
static int simp(int c){if(c==1||c==2)return 0;if(c==3)return 1;return -1;}

// Content width = window - padding*2
static float CW() { return ImGui::GetWindowSize().x - kP*2; }

// ─── InputXYZ ───────────────────────────────────────────────────────
static bool InputXYZ(const char* id, float v[3]) {
    bool ch=false; ImGui::PushID(id);
    float w=CW(), gap=4, chW=(w-gap*2)/3;
    const char* L[]={"x","y","z"};
    for(int i=0;i<3;++i){
        if(i)ImGui::SameLine(0,gap);
        ImGui::SetNextItemWidth(chW);
        char f[16]; std::snprintf(f,16,"%s  %%.1f",L[i]);
        ImGui::PushID(i); if(ImGui::DragFloat("##v",&v[i],0.01f,0,0,f))ch=true; ImGui::PopID();
    }
    ImGui::PopID(); return ch;
}

// ─── InputRGB ───────────────────────────────────────────────────────
static bool InputRGB(const char* id, float col[3]) {
    bool ch=false; ImGui::PushID(id);
    float w=CW(), gap=4, sw=40, chW=(w-sw-gap*3)/3;
    const char* L[]={"R","G","B"};
    for(int i=0;i<3;++i){
        if(i)ImGui::SameLine(0,gap);
        ImGui::SetNextItemWidth(chW);
        char f[16]; std::snprintf(f,16,"%s  %%.0f",L[i]);
        ImGui::PushID(i); float v=col[i]*255;
        if(ImGui::DragFloat("##c",&v,1,0,255,f)){col[i]=v/255;ch=true;} ImGui::PopID();
    }
    ImGui::SameLine(0,gap);
    ImVec4 pv(col[0],col[1],col[2],1);
    if(ImGui::ColorButton("##sw",pv,ImGuiColorEditFlags_NoTooltip,{sw,sw}))ImGui::OpenPopup("##pk");
    if(ImGui::BeginPopup("##pk")){
        if(ImGui::ColorPicker3("##p",col,ImGuiColorEditFlags_NoSidePreview|ImGuiColorEditFlags_NoSmallPreview|ImGuiColorEditFlags_NoInputs))ch=true;
        ImGui::EndPopup();
    }
    ImGui::PopID(); return ch;
}

// ─── PillToggle ─────────────────────────────────────────────────────
static bool PillToggle(const char* id, bool* v) {
    ImGui::PushID(id); bool ch=false;
    float w=42,h=24,r=h/2;
    ImVec2 pos=ImGui::GetCursorScreenPos();
    ImDrawList* dl=ImGui::GetWindowDrawList();
    if(ImGui::InvisibleButton("##t",{w,h})){*v=!*v;ch=true;}
    dl->AddRectFilled(pos,{pos.x+w,pos.y+h},*v?ImGui::ColorConvertFloat4ToU32(kG100):ImGui::ColorConvertFloat4ToU32(kG20),r);
    float tr=10,tx=*v?pos.x+w-2-tr:pos.x+2+tr;
    dl->AddCircleFilled({tx,pos.y+h/2},tr,IM_COL32(255,255,255,255),16);
    ImGui::PopID(); return ch;
}

// ─── AccordionHeader ────────────────────────────────────────────────
static std::unordered_map<ImGuiID,bool> sA;
static bool AccordionHeader(const char* kr, const char* en) {
    ImGui::PushID(kr);
    ImGuiID aid=ImGui::GetID("##a");
    if(sA.find(aid)==sA.end())sA[aid]=true;
    float fW=ImGui::GetWindowSize().x, hH=56;
    ImVec2 pos=ImGui::GetCursorScreenPos();
    ImDrawList* dl=ImGui::GetWindowDrawList();
    dl->AddRectFilled(pos,{pos.x+fW,pos.y+hH},ImGui::ColorConvertFloat4ToU32(kG5));
    bool cl=ImGui::InvisibleButton("##ab",{fW,hH});
    if(cl)sA[aid]=!sA[aid];
    bool op=sA[aid];
    ImFont* fo=ImGui::GetFont();
    float tFS=ImGui::GetFontSize(), sFS=ImGui::GetFontSize()*0.85f;
    float tY=pos.y+(hH-tFS)/2, sY=pos.y+(hH-sFS)/2;
    dl->AddText(fo,tFS,{pos.x+kP,tY},ImGui::ColorConvertFloat4ToU32(kG100),kr);
    if(en&&en[0]){ImVec2 ts=fo->CalcTextSizeA(tFS,FLT_MAX,0,kr);dl->AddText(fo,sFS,{pos.x+kP+ts.x+4,sY},ImGui::ColorConvertFloat4ToU32(kG60),en);}
    float aX=pos.x+fW-kP,aY=pos.y+hH/2;float r=5;ImU32 aC=ImGui::ColorConvertFloat4ToU32(kG60);
    if(op){dl->PathLineTo({aX-r,aY+r*.4f});dl->PathLineTo({aX,aY-r*.4f});dl->PathLineTo({aX+r,aY+r*.4f});}
    else{dl->PathLineTo({aX-r,aY-r*.4f});dl->PathLineTo({aX,aY+r*.4f});dl->PathLineTo({aX+r,aY-r*.4f});}
    dl->PathStroke(aC,0,2);
    ImGui::PopID(); return op;
}

// ─── InlineSlider: fixed 60px label offset ──────────────────────────
static bool InlineSlider(const char* label, float* v, float vmin, float vmax) {
    ImGui::PushID(label);
    float contentW=CW(), labelOff=60, valW=36, gap=8;
    float sliderW=contentW-labelOff-gap-valW;
    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted(label);ImGui::PopStyleColor();
    ImGui::SameLine(kP+labelOff);
    ImGui::PushStyleColor(ImGuiCol_FrameBg,kG10);ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,kG10);ImGui::PushStyleColor(ImGuiCol_FrameBgActive,kG10);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,ImVec2(0,4));
    ImGui::SetNextItemWidth(sliderW);
    bool ch=ImGui::SliderFloat("##sl",v,vmin,vmax,"");
    ImGui::PopStyleVar();ImGui::PopStyleColor(3);
    ImGui::SameLine(kP+contentW-valW);
    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::Text("%.2f",*v);ImGui::PopStyleColor();
    ImGui::PopID();return ch;
}

// ─── SegmentTab ─────────────────────────────────────────────────────
static void SegmentTab(const char* id, const char* lA, const char* lB, int cur, int* out) {
    ImGui::PushID(id);
    ImVec2 cP=ImGui::GetCursorScreenPos();
    float cW=124,cH=40,pad=4,segH=32,segW=(cW-pad*3)/2;
    ImGui::GetWindowDrawList()->AddRectFilled(cP,{cP.x+cW,cP.y+cH},ImGui::ColorConvertFloat4ToU32(kG10),8);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,8);ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize,0);
    float textH=ImGui::GetFontSize();float fpy=(segH-textH)*0.5f;
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,{0,fpy});
    auto seg=[&](const char* l,int idx,float x){
        bool act=(idx==cur);
        ImGui::SetCursorScreenPos({x,cP.y+pad});
        ImGui::PushStyleColor(ImGuiCol_Button,act?kW:ImVec4(0,0,0,0));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,act?kW:ImVec4(1,1,1,.5f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,kW);
        ImGui::PushStyleColor(ImGuiCol_Text,act?kG90:kG50);
        if(ImGui::Button(l,{segW,segH}))*out=idx;
        ImGui::PopStyleColor(4);
    };
    seg(lA,0,cP.x+pad);seg(lB,1,cP.x+pad+segW+pad);
    ImGui::PopStyleVar(3);
    // DON'T move cursor — caller manages positioning
    ImGui::PopID();
}

// ─── Card ───────────────────────────────────────────────────────────
static bool CardButton(const char* id, const char* title, void(*ico)(ImDrawList*,float,float,float)) {
    ImGui::PushID(id);
    float w=CW(),h=56;
    ImVec2 cP=ImGui::GetCursorScreenPos();
    ImDrawList* dl=ImGui::GetWindowDrawList();
    bool cl=ImGui::InvisibleButton("##c",{w,h});bool hv=ImGui::IsItemHovered();
    dl->AddRectFilled(cP,{cP.x+w,cP.y+h},ImGui::ColorConvertFloat4ToU32(hv?kG10:kG5),8);
    if(ico)ico(dl,cP.x+16,cP.y+(h-36)/2,36);
    ImFont* f=ImGui::GetFont();float fs=ImGui::GetFontSize();
    dl->AddText(f,fs,{cP.x+16+36+12,cP.y+(h-fs)/2},ImGui::ColorConvertFloat4ToU32(kG100),title);
    ImGui::PopID();return cl;
}
static void IcoPlus(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2;ImU32 co=ImGui::ColorConvertFloat4ToU32(kG40);d->AddLine({c-8,m},{c+8,m},co,1.5f);d->AddLine({c,m-8},{c,m+8},co,1.5f);}
static void IcoCube(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2,z=10;d->AddQuadFilled({c,m-z},{c+z,m-z/2},{c,m},{c-z,m-z/2},IM_COL32(224,229,236,255));d->AddQuadFilled({c-z,m-z/2},{c,m},{c,m+z},{c-z,m+z/2},IM_COL32(195,202,210,255));d->AddQuadFilled({c,m},{c+z,m-z/2},{c+z,m+z/2},{c,m+z},IM_COL32(210,217,224,255));}
static void IcoSphere(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2;d->AddCircleFilled({c,m},11,IM_COL32(220,226,233,255),24);d->AddCircleFilled({c-3,m-3},5,IM_COL32(240,243,247,255),12);}
static void IcoCyl(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2;d->AddRectFilled({c-8,m-9},{c+8,m+9},IM_COL32(220,226,233,255));d->AddEllipseFilled({c,m-9},{8,4},IM_COL32(210,217,224,255),0,16);d->AddEllipseFilled({c,m+9},{8,4},IM_COL32(210,217,224,255),0,16);}
static void IcoPlane(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2;ImVec2 p[4]={{c,m-8},{c+14,m},{c,m+8},{c-14,m}};d->AddConvexPolyFilled(p,4,IM_COL32(215,222,229,255));}

// DrawTrash icon helper
static void DrawTrash(ImDrawList* dl, float cx, float cy) {
    ImU32 ic=ImGui::ColorConvertFloat4ToU32(kG60);
    dl->AddRect({cx-6,cy-4},{cx+6,cy+8},ic,2,0,1.5f);
    dl->AddLine({cx-8,cy-4},{cx+8,cy-4},ic,1.5f);
    dl->AddLine({cx-3,cy-4},{cx-3,cy-7},ic,1.5f);dl->AddLine({cx-3,cy-7},{cx+3,cy-7},ic,1.5f);dl->AddLine({cx+3,cy-7},{cx+3,cy-4},ic,1.5f);
    dl->AddLine({cx-2,cy-1},{cx-2,cy+5},ic,1);dl->AddLine({cx,cy-1},{cx,cy+5},ic,1);dl->AddLine({cx+2,cy-1},{cx+2,cy+5},ic,1);
}

// ═════════════════════════════════════════════════════════════════════
void drawMeshInspectorWindow(MeshInspectorWindowState& st, const MeshInspectorTarget& t) {
    if(!st.open)return;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,0);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize,0);
    if(!ImGui::Begin("물체",nullptr,ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoSavedSettings|ImGuiWindowFlags_NoTitleBar)){
        ImGui::PopStyleVar(2);ImGui::End();return;}

    float winW = ImGui::GetWindowSize().x;

    // ═══ Point panel ═════════════════════════════════════════════════
    if(t.point_panel){
        ImGui::Dummy({0,kP});ImGui::Indent(kP);
        ImGui::Text("점 선택: 물체 %d / 정점 %d",t.point_obj,t.point_vert);
        ImGui::Unindent(kP);ImGui::Separator();ImGui::Dummy({0,kP});ImGui::Indent(kP);
        bool fx=t.point_fixed;if(ImGui::Checkbox("점 고정",&fx)&&t.on_point_set_fixed)t.on_point_set_fixed(fx);
        ImGui::Dummy({0,12});
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("위치");ImGui::PopStyleColor();ImGui::Dummy({0,4});
        float p[3]={t.point_position[0],t.point_position[1],t.point_position[2]};
        if(InputXYZ("pp",p)&&t.on_point_move)t.on_point_move(p[0],p[1],p[2]);
        ImGui::Dummy({0,12});
        {bool ra=t.point_ref_active;if(ra){ImGui::PushStyleColor(ImGuiCol_Button,kG90);ImGui::PushStyleColor(ImGuiCol_Text,kW);}
        if(ImGui::Button(ra?"참조점 선택 중...":"다른 점 위치 참조",{CW(),40})&&t.on_point_ref_toggle)t.on_point_ref_toggle();
        if(ra)ImGui::PopStyleColor(2);
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextWrapped("다른 점을 클릭하면 이 점이 따라갑니다.");ImGui::PopStyleColor();}
        ImGui::Unindent(kP);ImGui::Separator();ImGui::Dummy({0,kP});ImGui::Indent(kP);
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("연결된 참조점");ImGui::PopStyleColor();
        if(t.point_ref_constraints.empty()){ImGui::PushStyleColor(ImGuiCol_Text,kG50);ImGui::TextUnformatted("(없음)");ImGui::PopStyleColor();}
        else{for(size_t i=0;i<t.point_ref_constraints.size();++i){auto&e=t.point_ref_constraints[i];ImGui::PushID((int)i);ImGui::Text(e.selected_is_follower?"이 점->물체%d/정점%d":"물체%d/정점%d->이 점",e.other_obj,e.other_vert);ImGui::SameLine();if(ImGui::SmallButton("제거")&&t.on_point_ref_remove)t.on_point_ref_remove((int)i);ImGui::PopID();}}
        ImGui::Unindent(kP);ImGui::PopStyleVar(2);ImGui::End();return;
    }

    // ═══ Empty state ═════════════════════════════════════════════════
    if(t.mesh_id<0||t.base_color==nullptr){
        ImGui::Dummy({0,kP});ImGui::Indent(kP);
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("물체 리스트");ImGui::PopStyleColor();
        ImGui::Dummy({0,40});
        {float aw=CW();
        auto ctr=[&](const char* tx,ImVec4 c){ImVec2 sz=ImGui::CalcTextSize(tx);ImGui::SetCursorPosX(kP+(aw-sz.x)/2);ImGui::PushStyleColor(ImGuiCol_Text,c);ImGui::TextUnformatted(tx);ImGui::PopStyleColor();};
        ctr("아직 물체가 없어요",kG100);ctr("추가할 물체를 클릭하여 선택하세요.",kG60);}
        ImGui::Dummy({0,40});ImGui::Dummy({0,4});
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("새 물체 추가");ImGui::PopStyleColor();ImGui::Dummy({0,8});
        if(CardButton("obj","OBJ 파일 불러오기",IcoPlus)&&t.on_request_add_import)t.on_request_add_import();ImGui::Dummy({0,4});
        if(CardButton("cube","정육면체",IcoCube)&&t.on_request_add_cube)t.on_request_add_cube();ImGui::Dummy({0,4});
        if(CardButton("sphere","구",IcoSphere)&&t.on_request_add_sphere)t.on_request_add_sphere();ImGui::Dummy({0,4});
        if(CardButton("cyl","원기둥",IcoCyl)&&t.on_request_add_cylinder)t.on_request_add_cylinder();ImGui::Dummy({0,4});
        if(CardButton("plane","평면",IcoPlane)&&t.on_request_add_plane)t.on_request_add_plane();
        ImGui::Unindent(kP);ImGui::PopStyleVar(2);ImGui::End();return;
    }

    // ═══ Object selected ═════════════════════════════════════════════

    // Header: NO InvisibleButton — draw bg, then place real widgets directly
    {
        float hH=80;
        ImVec2 hP=ImGui::GetCursorScreenPos();
        ImDrawList* dl=ImGui::GetWindowDrawList();
        dl->AddRectFilled(hP,{hP.x+winW,hP.y+hH},IM_COL32(255,255,255,255));

        // Title text
        char hdr[64];std::snprintf(hdr,64,"Mesh #%d",t.mesh_id);
        ImFont* fo=ImGui::GetFont();float tFS=ImGui::GetFontSize();
        dl->AddText(fo,tFS,{hP.x+kP,hP.y+(hH-tFS)/2},ImGui::ColorConvertFloat4ToU32(kG100),hdr);

        float rX=hP.x+winW-kP;

        // Trash button: 40x40
        if(t.on_delete){
            float bW=40,bH=40,bX=rX-bW,bY=hP.y+(hH-bH)/2;
            ImGui::SetCursorScreenPos({bX,bY});
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,8);ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize,0);
            ImGui::PushStyleColor(ImGuiCol_Button,kG10);ImGui::PushStyleColor(ImGuiCol_ButtonHovered,kG20);ImGui::PushStyleColor(ImGuiCol_ButtonActive,kG20);
            if(ImGui::Button("##tr",{bW,bH}))t.on_delete(t.mesh_id);
            ImGui::PopStyleColor(3);ImGui::PopStyleVar(2);
            DrawTrash(dl,bX+20,bY+20);
            rX=bX-8;
        }

        // Segment tab: 124x40 — use simple buttons, NOT SegmentTab helper
        if(t.current_behavior_index>=0&&t.on_behavior_change){
            int sI=simp(t.current_behavior_index);
            float tabW=124,tabH=40,pad=4,segH=32;
            float segW=(tabW-pad*3)/2;
            float tabX=rX-tabW,tabY=hP.y+(hH-tabH)/2;
            // Tab container bg
            dl->AddRectFilled({tabX,tabY},{tabX+tabW,tabY+tabH},ImGui::ColorConvertFloat4ToU32(kG10),8);

            float textH=ImGui::GetFontSize();float fpy=(segH-textH)*0.5f;
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,8);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize,0);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,{0,fpy});

            // 강체 button
            bool rigidActive=(sI==1);
            ImGui::SetCursorScreenPos({tabX+pad,tabY+pad});
            ImGui::PushStyleColor(ImGuiCol_Button,rigidActive?kW:ImVec4(0,0,0,0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,rigidActive?kW:ImVec4(1,1,1,.5f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,kW);
            ImGui::PushStyleColor(ImGuiCol_Text,rigidActive?kG90:kG50);
            if(ImGui::Button("강체",{segW,segH})){
                t.on_behavior_change(t.mesh_id,3); // 3=Rigid
            }
            ImGui::PopStyleColor(4);

            // 옷감 button
            bool clothActive=(sI==0);
            ImGui::SetCursorScreenPos({tabX+pad+segW+pad,tabY+pad});
            ImGui::PushStyleColor(ImGuiCol_Button,clothActive?kW:ImVec4(0,0,0,0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,clothActive?kW:ImVec4(1,1,1,.5f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,kW);
            ImGui::PushStyleColor(ImGuiCol_Text,clothActive?kG90:kG50);
            if(ImGui::Button("옷감",{segW,segH})){
                t.on_behavior_change(t.mesh_id,clothBI(t.grid_eligible));
            }
            ImGui::PopStyleColor(4);
            ImGui::PopStyleVar(3);
        }

        // Advance cursor past header
        ImGui::SetCursorScreenPos({hP.x, hP.y+hH});
    }

    // ─── 변환 Transform ──────────────────────────────────────────────
    if(t.transform_position||t.rotation_wxyz||t.scale){
        if(AccordionHeader("변환","Transform")){
            ImGui::Dummy({0,kP});ImGui::Indent(kP);
            if(t.transform_position&&t.on_translate){
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("위치");ImGui::PopStyleColor();ImGui::Dummy({0,4});
                float pp[3]={t.transform_position->x,t.transform_position->y,t.transform_position->z};
                if(InputXYZ("pos",pp))t.on_translate(t.mesh_id,tinym::vec3(pp[0],pp[1],pp[2]));
                ImGui::Dummy({0,20});
            }
            if(t.rotation_wxyz&&t.on_rotate){
                const float cw[4]={t.rotation_wxyz[0],t.rotation_wxyz[1],t.rotation_wxyz[2],t.rotation_wxyz[3]};
                float deg[3];quatWxyzToEulerXYZDeg(cw,deg);
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("회전 (각도)");ImGui::PopStyleColor();ImGui::Dummy({0,4});
                if(InputXYZ("rot",deg)){float nw[4];eulerXYZDegToQuatWxyz(deg,nw);t.on_rotate(t.mesh_id,nw[0],nw[1],nw[2],nw[3]);}
                ImGui::Dummy({0,20});
            }
            if(t.scale&&t.on_scale){
                float s[3]={t.scale->x,t.scale->y,t.scale->z};
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("스케일");ImGui::PopStyleColor();ImGui::Dummy({0,4});
                if(InputXYZ("scale",s))t.on_scale(t.mesh_id,tinym::vec3(s[0],s[1],s[2]));
            }
            ImGui::Unindent(kP);ImGui::Dummy({0,kP});
        }
    }

    // ─── 머터리얼 Material ───────────────────────────────────────────
    bool hM=t.metallic&&t.roughness&&t.specular_weight&&t.emission_color&&t.on_material_edit;
    auto fM=[&](){if(!hM)return;t.on_material_edit(t.mesh_id,*t.base_color,*t.metallic,*t.roughness,*t.specular_weight,*t.emission_color);};
    if(AccordionHeader("머터리얼","Material")){
        ImGui::Dummy({0,kP});ImGui::Indent(kP);
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("RGB");ImGui::PopStyleColor();ImGui::Dummy({0,4});
        if(InputRGB("bc",t.base_color->v))fM();
        if(hM){
            ImGui::Dummy({0,24});
            ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("PBR 속성");ImGui::PopStyleColor();ImGui::Dummy({0,8});
            if(InlineSlider("금속성",t.metallic,0,1))fM();ImGui::Dummy({0,8});
            if(InlineSlider("거칠기",t.roughness,0,1))fM();ImGui::Dummy({0,8});
            if(InlineSlider("스페큘러",t.specular_weight,0,1))fM();
            ImGui::Dummy({0,24});
            ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("발광 색상");ImGui::PopStyleColor();ImGui::Dummy({0,4});
            if(InputRGB("em",t.emission_color->v))fM();
        }
        ImGui::Unindent(kP);ImGui::Dummy({0,kP});
    }

    // ─── 팽팽함 ──────────────────────────────────────────────────────
    if(t.cloth_stiffness_scale&&t.on_cloth_stiffness_scale){
        float sc=*t.cloth_stiffness_scale;float k=(sc>0)?std::log10(sc):0;if(k<-2)k=-2;if(k>2)k=2;
        ImGui::Dummy({0,kP});ImGui::Indent(kP);
        if(InlineSlider("팽팽함",&k,-2,2)){float ns=std::pow(10.f,k);*t.cloth_stiffness_scale=ns;t.on_cloth_stiffness_scale(t.mesh_id,ns);}
        ImGui::Unindent(kP);ImGui::Dummy({0,kP});
    }

    // ─── 환경 Environment ────────────────────────────────────────────
    if(t.apply_gravity||t.apply_wind){
        if(AccordionHeader("환경","Environment")){
            ImGui::Dummy({0,kP});ImGui::Indent(kP);
            auto fE=[&](){if(!t.on_env_toggle_change)return;bool g=t.apply_gravity?*t.apply_gravity:true;bool w=t.apply_wind?*t.apply_wind:true;t.on_env_toggle_change(t.mesh_id,g,w);};
            if(t.apply_gravity){
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("중력");ImGui::PopStyleColor();
                ImGui::SameLine(kP+CW()-42);if(PillToggle("##gv",t.apply_gravity))fE();ImGui::Dummy({0,12});
            }
            if(t.apply_wind){
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("바람");ImGui::PopStyleColor();
                ImGui::SameLine(kP+CW()-42);if(PillToggle("##wd",t.apply_wind))fE();ImGui::Dummy({0,12});
            }
            ImGui::Unindent(kP);ImGui::Dummy({0,kP});
        }
    }

    ImGui::PopStyleVar(2);ImGui::End();
}
} // namespace mesh_inspector
