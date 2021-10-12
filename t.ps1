foreach($item in Get-ChildItem .\data\questions\*.json)
{
    Write-Host ('# ' + $item.Name)

    $q_name = $item.Name
    $a_name = $q_name.Replace("question2-", "answer2-")

    Start-Sleep 5

    $time = Measure-Command {
        Get-Content ('.\data\questions\' + $q_name) | .\build\Release\sandrokottos > ('.\data\answers\' + $a_name)
    }

    Write-Host ('' + $time.TotalSeconds + ' sec.')
}
