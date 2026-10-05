package com.iqoo.breadcrumb

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class M0SanityTest {

    @Test
    fun testPackageConfiguration() {
        val expectedPackage = "com.iqoo.breadcrumb"
        assertEquals("com.iqoo.breadcrumb", expectedPackage)
    }

    @Test
    fun testTargetVersionCode() {
        val expectedVersion = 1
        assertEquals(1, expectedVersion)
    }
}
