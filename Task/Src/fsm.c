/************************** Dongguan-University of Technology -ACE**************************
 * @file fsm.c
 * @brief 
 * @author pansyhou���Ļ� (1677195845lyb@gmail.com)
 * @version 1.0 
 * @date 2022-07-12
 * 
 * 
 * @history
 * <table>
 * Date       Version Author Description
 *             1.0   �żһ�
 *  2022-07-12 1.1   ���Ļ� 
 *  2022-10-19 1.2   ��C
 * @verbatim 



               AAA                              CCCCCCCCCCCCCE               EEEEEEEEEEEEEEEEEEEEE
              A:::A                          CCC::::::::::::CE               ::::::::::::::::::::E
             A:::::A                       CC:::::::::::::::CE               ::::::::::::::::::::E
            A:::::::A                     C:::::CCCCCCCC::::CE               E::::::EEEEEEEEE::::E
           A:::::::::A                   C:::::C       CCCCCC                E:::::E       EEEEEE
          A:::::A:::::A                 C:::::C                              E:::::E
         A:::::A A:::::A                C:::::C                              E::::::EEEEEEEEEE
        A:::::A   A:::::A               C:::::C                              E:::::::::::::::E
       A:::::A     A:::::A              C:::::C                              E:::::::::::::::E
      A:::::AAAAAAAAA:::::A             C:::::C                              E::::::EEEEEEEEEE
     A:::::::::::::::::::::A            C:::::C                              E:::::E
    A:::::AAAAAAAAAAAAA:::::A            C:::::C       CCCCCC                E:::::E       EEEEEE
   A:::::A             A:::::A            C:::::CCCCCCCC::::CE               E::::::EEEEEEEE:::::E
  A:::::A               A:::::A            CC:::::::::::::::CE               ::::::::::::::::::::E
 A:::::A                 A:::::A             CCC::::::::::::CE               ::::::::::::::::::::E
AAAAAAA                   AAAAAAA               CCCCCCCCCCCCCE               EEEEEEEEEEEEEEEEEEEEE





 * @endverbatim
************************** Dongguan-University of Technology -ACE***************************/

#include "fsm.h"
/*****************************��ݸ��ѧԺACEʵ���� *****************************
 * @brief ״̬������
 * 
 * @param fsm 
 * @param s1 �󲦸���ֵ
 * @param s2 ��
 * ps����Щ����ָ�붼���ڸ��Եĳ�ʼ��������ָ���Լ�����״̬�ĺ���
*******************************��ݸ��ѧԺACEʵ���� *****************************/
void FSM_Deal(FSM_t *fsm, uint8_t s1, uint8_t s2)
{
    /*������ж�,�߽籣��*/
    if (s1 <= 0 || s1 > State_Line || s2 <= 0 || s2 > State_Column)
    {
        return;
    }

    /*״ָ̬��*/
    fsm->Current_State = &fsm->State_Table[s1 - 1][s2 - 1];

    /*״̬�仯*/
    if (fsm->State_Change(fsm->Last_State, fsm->Current_State) == 1)
    {
        fsm->Current_State->State_Prepare();
    }

    /*�����ϴ�״̬*/
    fsm->Last_State = fsm->Current_State;

    /*ִ��״̬*/
    fsm->Current_State->State_Process();

    /*ִ��ʵ����Ϊ*/
    fsm->Current_State->Behavior_Process();
}


/*****************************��ݸ��ѧԺACEʵ���� *****************************
 * @brief ״̬���������ң�������Ҳ�����״̬
 * 
 * @param L_Sta Last_State�ϴ�״̬
 * @param C_Sta Current_StateĿǰ״̬
 * @return unsigned char 
*******************************��ݸ��ѧԺACEʵ���� *****************************/
unsigned char StateChange(State_t *L_Sta, State_t *C_Sta)
{
    return (L_Sta != C_Sta ? (1) : (0));
}
