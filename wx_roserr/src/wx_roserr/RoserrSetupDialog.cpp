#include "RoserrSetupDialog.h"

RoserrSetupDialog::RoserrSetupDialog( wxWindow* parent )
:
RoserrSetupDialogBase( parent )
{

}

void RoserrSetupDialog::OnCancel( wxCommandEvent& event )
{
	EndModal( wxID_CANCEL );
}

void RoserrSetupDialog::OnOk( wxCommandEvent& event )
{
	EndModal( wxID_OK );
}

bool RoserrSetupDialog::GetMonochrome()
{
	return m_MonochromeCB->GetValue();
}

bool RoserrSetupDialog::GetWordWrap()
{
	return m_WordWrapCB->GetValue();
}
