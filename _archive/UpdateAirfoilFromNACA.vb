Imports Inventor
Imports System
Imports System.Collections.Generic

' iLogic Rule: UpdateAirfoilFromNACA
' Inventor Pro 2024+
' Creates a LIGHT sketch: keeps only a SketchFixedSpline (no fit-point sketch points)

Sub Main()

    Dim doc As Inventor.Document = ThisDoc.Document
    Dim partDoc As Inventor.PartDocument = TryCast(doc, Inventor.PartDocument)
    If partDoc Is Nothing Then
        MessageBox.Show("This rule must be run in a Part (.ipt) document.", "NACA")
        Exit Sub
    End If

    Dim cd As Inventor.PartComponentDefinition = partDoc.ComponentDefinition
    Dim uom As Inventor.UnitsOfMeasure = partDoc.UnitsOfMeasure
    Dim tg As Inventor.TransientGeometry = ThisApplication.TransientGeometry

    ' --- Normalize NACA code (Inventor may turn 0006 into 6) ---
    Dim rawCode As String = CStr(Parameter("vertical_NACA_code")).Trim()
    Dim code As String = NormalizeNacaCode(rawCode)
    If code = "" Then
        MessageBox.Show("vertical_NACA_code must be a number 0..9999 (e.g. 6, 0006, 2410).", "NACA")
        Exit Sub
    End If

    ' chord in mm (parameter c_root is in mm)
    Dim chordMm As Double = CDbl(Parameter("c_root"))
    If chordMm <= 0 Then
        MessageBox.Show("c_root must be > 0", "NACA")
        Exit Sub
    End If

    ' Work in cm for sketch coordinates
    Dim chordCm As Double = uom.ConvertUnits(chordMm, UnitsTypeEnum.kMillimeterLengthUnits, UnitsTypeEnum.kCentimeterLengthUnits)

    Dim n As Integer = CInt(Parameter("airfoil_n_points"))
    If n < 25 Then n = 25
    If n > 1500 Then n = 1500

    ' NACA 4-digit: MPXX
    Dim m As Double = CDbl(Mid(code, 1, 1)) / 100.0
    Dim p As Double = CDbl(Mid(code, 2, 1)) / 10.0
    Dim t As Double = CDbl(Mid(code, 3, 2)) / 100.0

    ' Get/Create sketch on XY plane
    Dim sk As Inventor.PlanarSketch = GetOrCreateSketchXY(cd, "AirfoilSketch")

    ' Clear existing geometry
    ClearSketch(sk)

    ' Build outline points in sketch space (cm)
    Dim outline As List(Of Inventor.Point2d) = BuildNACAOutlinePoints(tg, m, p, t, n, chordCm)
    If outline Is Nothing OrElse outline.Count < 10 Then
        MessageBox.Show("Not enough points generated.", "NACA")
        Exit Sub
    End If

    ' Create fit points collection (Point2d) for SketchSplines.Add
    Dim fitPts As Inventor.ObjectCollection = ThisApplication.TransientObjects.CreateObjectCollection()
    For Each pt As Inventor.Point2d In outline
        fitPts.Add(pt)
    Next

    Dim tmpSpline As Inventor.SketchSpline = Nothing

    Try
        ' 1) Create a temporary (heavy) spline
        tmpSpline = sk.SketchSplines.Add(fitPts)

        ' 2) Copy its BSpline geometry and create a FIXED spline (light)
        Dim nurbsGeom As Object = tmpSpline.Geometry   ' BSplineCurve2d (as Object avoids type-issues in iLogic)
        Dim fixedSpline As Inventor.SketchFixedSpline = sk.SketchFixedSplines.Add(nurbsGeom)

        ' 3) Delete the temporary spline (removes fit-point baggage)
        tmpSpline.Delete()
        tmpSpline = Nothing

    Catch ex As Exception
        ' Clean up if temp spline was created
        Try
            If Not tmpSpline Is Nothing Then
                tmpSpline.Delete()
            End If
        Catch
            ' ignore cleanup errors
        End Try

        MessageBox.Show("Failed creating fixed spline: " & ex.Message, "NACA")
        Exit Sub
    End Try

    ' Optional: close the trailing edge with a small line if endpoints don't coincide
    Try
        Dim p0 As Inventor.Point2d = outline(0)
        Dim pN As Inventor.Point2d = outline(outline.Count - 1)
        Dim dx As Double = p0.X - pN.X
        Dim dy As Double = p0.Y - pN.Y
        Dim dist As Double = Math.Sqrt(dx * dx + dy * dy)

        If dist > 1.0E-6 Then
            sk.SketchLines.AddByTwoPoints(pN, p0)
        End If
    Catch
        ' If closing line fails, keep spline anyway
    End Try

    partDoc.Update()

End Sub


' ----------------------------
' Normalize NACA code: "6" -> "0006", "NACA2410" -> "2410"
' ----------------------------
Function NormalizeNacaCode(raw As String) As String
    If raw Is Nothing Then Return ""

    Dim s As String = raw.Trim()
    s = s.Replace(" ", "")

    Dim digitsOnly As String = ""
    For i As Integer = 1 To Len(s)
        Dim ch As String = Mid(s, i, 1)
        If ch >= "0" AndAlso ch <= "9" Then digitsOnly &= ch
    Next

    If digitsOnly = "" Then Return ""
    If Not IsNumeric(digitsOnly) Then Return ""

    Dim nn As Integer = CInt(digitsOnly)
    If nn < 0 OrElse nn > 9999 Then Return ""

    Return nn.ToString("0000")
End Function


Function GetOrCreateSketchXY(cd As Inventor.PartComponentDefinition, sketchName As String) As Inventor.PlanarSketch
    For Each s As Inventor.PlanarSketch In cd.Sketches
        If String.Compare(s.Name, sketchName, True) = 0 Then
            Return s
        End If
    Next

    Dim wp As Inventor.WorkPlane = GetXYWorkPlane(cd)
    Dim sk As Inventor.PlanarSketch = cd.Sketches.Add(wp)
    sk.Name = sketchName
    Return sk
End Function


Function GetXYWorkPlane(cd As Inventor.PartComponentDefinition) As Inventor.WorkPlane
    ' Prefer by name, fallback to index
    Try
        Return cd.WorkPlanes.Item("XY Plane")
    Catch
        Return cd.WorkPlanes.Item(3)
    End Try
End Function


Sub ClearSketch(sk As Inventor.PlanarSketch)
    ' Delete dims, constraints, then entities (reverse order)
    ' (All Try/Catch; no On Error)

    Try
        For i As Integer = sk.DimensionConstraints.Count To 1 Step -1
            Try
                sk.DimensionConstraints.Item(i).Delete()
            Catch
            End Try
        Next
    Catch
    End Try

    Try
        For i As Integer = sk.GeometricConstraints.Count To 1 Step -1
            Try
                sk.GeometricConstraints.Item(i).Delete()
            Catch
            End Try
        Next
    Catch
    End Try

    Try
        For i As Integer = sk.SketchEntities.Count To 1 Step -1
            Try
                sk.SketchEntities.Item(i).Delete()
            Catch
            End Try
        Next
    Catch
    End Try
End Sub


Function BuildNACAOutlinePoints(
    tg As Inventor.TransientGeometry,
    m As Double, p As Double, t As Double,
    n As Integer, chordCm As Double
) As List(Of Inventor.Point2d)

    ' Standard 4-digit thickness formula
    ' a4 = -0.1015 => "closed TE" coefficient commonly used in many implementations
    Dim a0 As Double = 0.2969
    Dim a1 As Double = -0.1260
    Dim a2 As Double = -0.3516
    Dim a3 As Double = 0.2843
    Dim a4 As Double = -0.1015

    Dim xs As List(Of Double) = CosineSpacing(n)

    Dim upper As New List(Of Double())()
    Dim lower As New List(Of Double())()

    For Each x As Double In xs

        Dim yt As Double =
            5.0 * t * (a0 * Math.Sqrt(Math.Max(x, 0.0)) + a1 * x + a2 * x ^ 2 + a3 * x ^ 3 + a4 * x ^ 4)

        Dim yc As Double = 0.0
        Dim dyc_dx As Double = 0.0

        If m = 0.0 OrElse p = 0.0 Then
            yc = 0.0
            dyc_dx = 0.0
        ElseIf x < p Then
            yc = (m / (p ^ 2)) * (2.0 * p * x - x ^ 2)
            dyc_dx = (2.0 * m / (p ^ 2)) * (p - x)
        Else
            yc = (m / ((1.0 - p) ^ 2)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x ^ 2)
            dyc_dx = (2.0 * m / ((1.0 - p) ^ 2)) * (p - x)
        End If

        Dim theta As Double = Math.Atan(dyc_dx)

        Dim xu As Double = x - yt * Math.Sin(theta)
        Dim yu As Double = yc + yt * Math.Cos(theta)

        Dim xl As Double = x + yt * Math.Sin(theta)
        Dim yl As Double = yc - yt * Math.Cos(theta)

        upper.Add(New Double() {xu, yu})
        lower.Add(New Double() {xl, yl})
    Next

    Dim outline As New List(Of Inventor.Point2d)()

    ' Upper reversed: TE -> LE
    For i As Integer = upper.Count - 1 To 0 Step -1
        outline.Add(tg.CreatePoint2d(upper(i)(0) * chordCm, upper(i)(1) * chordCm))
    Next

    ' Lower forward: skip LE duplicate (index 0), include TE at end
    For i As Integer = 1 To lower.Count - 1
        outline.Add(tg.CreatePoint2d(lower(i)(0) * chordCm, lower(i)(1) * chordCm))
    Next

    Return outline
End Function


Function CosineSpacing(n As Integer) As List(Of Double)
    Dim xs As New List(Of Double)()
    For i As Integer = 0 To n - 1
        Dim beta As Double = Math.PI * i / (n - 1)
        xs.Add(0.5 * (1.0 - Math.Cos(beta)))
    Next
    Return xs
End Function
